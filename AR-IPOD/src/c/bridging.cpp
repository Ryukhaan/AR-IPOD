//
//  volumeInit.cpp
//  AR-IPOD
//
//  Created by Remi Decelle on 27/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

extern "C" {
    #include "bridging.hpp"
}

#include <simd/common.h>
#include <simd/vector_types.h>
#include <simd/vector_make.h>
#include <simd/vector.h>
#include <simd/conversion.h>
#include <simd/matrix.h>
#include <vector>
//#include <omp.h>

#include "linear_algebra.hpp"
#include "marching_cube.hpp"
#include "io.hpp"

using namespace std;

typedef struct Voxel {
    float sdf;
    unsigned char weight;
} Voxel;

/*
 * LOCAL FUNCTIONS
 */

/**
 * Update voxel sdf and weight.
 */
inline void update_voxel(Voxel* voxels, const float sdf, const int weight, const int index) {
    float old_sdf      = voxels[index].sdf;
    float old_weight   = voxels[index].weight;
    float new_weight   = old_weight + weight;
    float old_product  = old_weight * old_sdf;
    float new_product  = sdf * weight;
    float new_sdf      = (new_product + old_product ) / new_weight;
    
    voxels[index].sdf     = new_sdf;
    voxels[index].weight  = simd_min(new_weight, 100);
}

/**
 * Reset voxel
 */
inline void reset_voxel(Voxel * voxels, const int i) {
    voxels[i].sdf = 9999;
    voxels[i].weight = 0;
}

/*
inline void carveVoxel(float* sdfs, unsigned char* weight, const int index) {
    updateVoxel(sdfs, weight, 0.0, 1, index);
}
*/
                                          
/*
 * GLOBAL FUNCTION (hpp)
 */
void bridge_initializeCentroids(void* centroids, int size, float resolution) {
    //int this_thread = omp_get_thread_num();
    //int num_threads = omp_get_num_threads();
    int count = size * size * size;
    int square = size * size;
#pragma omp parallel for shared(centroids) num_threads(num_threads)
    for(int i = 0; i<count; i++) {
        float x = i / square;
        float remainder = i % square;
        float y = remainder / size;
        float z = (float)(((int) remainder) % size);
        ((simd::float3*) centroids)[i][0] = integer_to_global(x, size, resolution);
        ((simd::float3*) centroids)[i][1] = integer_to_global(y, size, resolution);
        ((simd::float3*) centroids)[i][2] = integer_to_global(z, size, resolution);
    }
}

unsigned long bridge_extractMesh(void* triangles,
                            const float* voxels,
                            const void* centroids,
                            int edgeTable[256],
                            int triTable[4096],
                            int n,
                            float isolevel) {
    unsigned long index = 0;
    int n2 = n * n;
//  int num_threads = omp_get_num_threads();
//#pragma omp parallel for shared(index) collapse(3) num_threads(num_threads)
    for (int i = 0; i<n-1; i++) {
        for (int j = 0; j<n-1; j++) {
            for (int k = 0; k<n-1; k++) {
                int i0 = i*n2 + j*n + (k+1);
                int i1 = (i+1)*n2 + j*n + (k+1);
                int i2 = (i+1)*n2 + j*n + k;
                int i3 = i*n2 + j*n + k;
                int i4 = i*n2 + (j+1)*n + (k+1);
                int i5 = (i+1)*n2 + (j+1)*n + (k+1);
                int i6 = (i+1)*n2 + (j+1)*n + k;
                int i7 = i*n2 + (j+1)*n + k;
                simd::float3 points[8] = {
                    ((simd::float3*) centroids)[i0],
                    ((simd::float3*) centroids)[i1],
                    ((simd::float3*) centroids)[i2],
                    ((simd::float3*) centroids)[i3],
                    ((simd::float3*) centroids)[i4],
                    ((simd::float3*) centroids)[i5],
                    ((simd::float3*) centroids)[i6],
                    ((simd::float3*) centroids)[i7]
                };
                float values[8] = {
                    fabs(voxels[i0]),
                    fabs(voxels[i1]),
                    fabs(voxels[i2]),
                    fabs(voxels[i3]),
                    fabs(voxels[i4]),
                    fabs(voxels[i5]),
                    fabs(voxels[i6]),
                    fabs(voxels[i7])
                };
                std::vector<simd::float3> temp = polygonise(points, values, isolevel, edgeTable, triTable);
                if (temp.size() == 0) continue;
                for (simd::float3 triangle : temp) {
                    ((simd::float3*) triangles)[index][0] = triangle[0];
                    ((simd::float3*) triangles)[index][1] = triangle[1];
                    ((simd::float3*) triangles)[index][2] = triangle[2];
//#pragma omp atomic
                    index++;
                }
            }
        }
    }
    return index;
}

int bridge_integrateDepthMap(const float* depthmap,
                              const void* centroids,
                              const void* camera_pose,
                              const void* intrisics,
                              void* voxels,
                              const int width,
                              const int height,
                             const int dimension,
                             const float resolution[3]) {
    // Instanciate all local variables
    int number_of_changes = 0;
    float delta = 0.3;
    float truncation = 0.3;
    float diag = 2.0 * sqrt(3.0) * resolution[0];
    float carving_distance = 0.5;
    int count = width * height;
    //int count = dimension * dimension * dimension;
    simd_float3x3 K = ((simd_float3x3 *) intrisics)[0];
    simd_float4x4 Rt = ((simd_float4x4 *) camera_pose)[0];
    simd_float3x3 Kinv = simd_inverse(K);
    simd::float3 translation = simd_make_float3(Rt.columns[3][0], Rt.columns[3][1], Rt.columns[3][2]);
    simd_float4x3 temp = simd_transpose(simd_matrix(Rt.columns[0], Rt.columns[1], Rt.columns[2]));
    simd_float3x3 rotation = simd_matrix_from_rows(temp.columns[0], temp.columns[1], temp.columns[2]);
    simd_float3 resolve = simd_make_float3(resolution[0], resolution[1], resolution[2]);
    // Compute observed voxels
    //simd_float3* observed = estimate_observed_voxels(depthmap, Rt, K, width, height);
    
    // Update each voxels
    for (int i = 0; i<count; i++) {
        /** Process by depthmap */
        float depth = depthmap[i];
        // Depth are in mm. Imo, having a 1nm threshhold as error is due to float conversion
        if (depth < 0.000001) continue;
        
        // Transfer global 3D into local camera coordinate
        simd_float3 local = unproject(simd_make_int2(i / width, i % width), depth, Kinv);
        simd_float3 global = simd_mul(rotation, local + translation);
        local = simd_mul(simd_transpose(rotation),  global - translation);
        float z = local.z;
        
        // Calculate which voxel will be updated
        simd::float3 ijk = trilinear_interpolation(mapping_global_to_voxel(global, dimension, resolve));
        int index = hash_function(ijk, dimension);
        if (index < 0) continue;
        
        // Project that local 3D point
        simd_int2 uv = project(local, K);
        if (uv.x > height || uv.x < 0) continue;
        if (uv.y > width || uv.y < 0) continue;
        
        float real_depth = depthmap[uv.x * width + uv.y];
        //float truncation = 2.0; // truncation distance
        float distance = z - real_depth;
        if (fabs(distance) < delta)
            update_voxel((Voxel *)voxels, distance, 1, index);
        else if (distance >= delta)
            update_voxel((Voxel *)voxels, delta, 1, index);
        else
            update_voxel((Voxel *)voxels, -delta, 1, index);
        /*
        simd::float3 centroid = ((simd::float3 *) centroids)[i];
        simd::float3 voxel_center = simd_mul(simd_transpose(rotation), centroid - translation);
        simd::float3 uvz = projectWithZ(voxel_center, K);
        
        if (uvz.x > height || uvz.x < 0) continue;
        if (uvz.y > width || uvz.y < 0) continue;
        if (uvz.z == 0) continue;
        
        float pixel_depth = uvz.z;
        float depth = depthmap[(int)uvz.x * width + (int)uvz.y];
        if (depth < 0.00001) continue;
        
        float distance = depth - pixel_depth;
        if (fabs(distance) < delta)
            update_voxel((Voxel *)voxels, distance, 1, i);
        else
            reset_voxel((Voxel *)voxels, i);
        */
    }
    return number_of_changes;
}

void bridge_exportMeshToPLY(const void* vectors,
                        const char* file_name,
                        int n) {
    save_meshing_ply_format((simd::float3 *) vectors, file_name, n);
}

void bridge_exportVolumeToPLY(const void* centroids,
                        const float* sdfs,
                        const char* file_name,
                        int size) {
    save_volume_ply_format((simd::float3 *) centroids, sdfs, file_name, size);
}
