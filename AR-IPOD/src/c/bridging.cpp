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
#include <unordered_map>
//#include <omp.h>

#include "linear_algebra.hpp"
#include "marching_cube.hpp"
#include "io.hpp"
#include "TSDF.hpp"

using namespace std;
                                          
/*
 * GLOBAL FUNCTION (hpp)
 */
void bridge_initializeCentroids(void* centroids, int size, float resolution) {
    //int this_thread = omp_get_thread_num();
    //int num_threads = omp_get_num_threads();
    int count = size * size * size;
    int square = size * size;
    //float offset = 0.5 * resolution * size ;
#pragma omp parallel for shared(centroids) num_threads(num_threads)
    for(int i = 0; i<count; i++) {
        int x = i / square;
        int remainder = i % square;
        int y = remainder / size;
        int z = (((int) remainder) % size);
        
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
                             const float resolution[3],
                             const float delta,
                             const float epsilon) {
    // Instanciate all local variables
    int number_of_changes = 0;
    //float delta = 0.3;
    //float epsilon = 0.15;
    int count = width * height;
    int size = pow(dimension, 3.0);
    
    // Relative camera variables
    simd_float3x3 K = ((simd_float3x3 *) intrisics)[0];
    simd_float4x4 Rt = ((simd_float4x4 *) camera_pose)[0];
    simd_float3x3 Kinv = simd_inverse(K);
    /*
    simd::float3 translation = simd_make_float3(Rt.columns[3][0], Rt.columns[3][1], Rt.columns[3][2]);
    simd_float4x3 temp = simd_transpose(simd_matrix(Rt.columns[0], Rt.columns[1], Rt.columns[2]));
    simd_float3x3 rotation = simd_matrix_from_rows(temp.columns[0], temp.columns[1], temp.columns[2]);
    */
    simd_float4x4 tRt = simd_transpose(Rt);
    simd_float4x3 Rtc = simd_matrix_from_rows(tRt.columns[0], tRt.columns[1], tRt.columns[2]);
    simd_float3x3 rotation = simd_matrix(Rtc.columns[0], Rtc.columns[1], Rtc.columns[2]);
    simd_float3 translation = simd_make_float3(Rtc.columns[3][0],Rtc.columns[3][1],Rtc.columns[3][2]);
    simd_float3 resolve = simd_make_float3(resolution[0], resolution[1], resolution[2]);
    
    for (int i = 0; i < count; i++) {
        float depth = depthmap[i];
        if (depth < 0.000001) continue;
        
        // Retrieve world coordinate point
        simd::float3 uv = simd_make_float3(i / width, i % width, 1);
        simd::float3 world_point = simd_mul(simd_transpose(rotation), simd_mul(Kinv, depth * uv) - translation);
        
        simd::float3 ijk = mapping_global_to_voxel(world_point, dimension, resolve);
        //simd::float3 nearest_centroid = trilinear_interpolation(ijk);
        int index = hash_function(ijk, dimension);
        if (index < 0 || index > size) continue;
        
        simd::float3 nearest_centroid = ((simd::float3 *) centroids)[index];
        float distance = depth - nearest_centroid.z;
        
        if (distance >= delta + epsilon && distance < nearest_centroid.z) {
            carving_voxel((Voxel *)voxels, index);
        }
        if (fabs(distance) <= delta) {
            update_voxel((Voxel *)voxels, distance, 1, index);
        }
    }
    /*
    int square = dimension * dimension;
    for (int i = 0; i<size; i++) {
        simd::float3 centroid = ((simd::float3 *) centroids)[i];
        int x = i / square;
        int remainder = i % square;
        int y = remainder / dimension;
        
        float depth = depthmap[x * width + y];
        if (depth < 0.000001) continue;
        
        float distance = (depth - centroid.z) / 2;
        if (fabs(distance) <= delta) {
            update_voxel((Voxel *)voxels, distance, 1, i);
        }
        else if (distance > delta) {
            update_voxel((Voxel *)voxels, delta, 1, i);
        }
        else {
            update_voxel((Voxel *)voxels, -delta, 1, i);
        }
    }
    */
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
