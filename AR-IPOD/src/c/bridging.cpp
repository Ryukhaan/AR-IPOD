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
    int count = pow(size, 3.0);
    int square = pow(size, 2.0);
    float offset = (resolution * 0.5);
#pragma omp parallel for shared(centroids) num_threads(num_threads)
    for(int i = 0; i<count; i++) {
        int x = i / square;
        int remainder = i % square;
        int y = remainder / size;
        int z = (float)(((int) remainder) % size);
        ((simd::float3*) centroids)[i][0] = integer_to_global(x, size, resolution) - offset;
        ((simd::float3*) centroids)[i][1] = integer_to_global(y, size, resolution) - offset;
        ((simd::float3*) centroids)[i][2] = integer_to_global(z, size, resolution); //- offset;
    }
}

unsigned long bridge_extractMesh(void* triangles,
                            const float* voxels,
                            //const void* centroids,
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
                simd::float3 c0 = create_centroid(i0, n, 4.0, n2, 2.0);
                simd::float3 c1 = create_centroid(i1, n, 4.0, n2, 2.0);
                simd::float3 c2 = create_centroid(i2, n, 4.0, n2, 2.0);
                simd::float3 c3 = create_centroid(i3, n, 4.0, n2, 2.0);
                simd::float3 c4 = create_centroid(i4, n, 4.0, n2, 2.0);
                simd::float3 c5 = create_centroid(i5, n, 4.0, n2, 2.0);
                simd::float3 c6 = create_centroid(i6, n, 4.0, n2, 2.0);
                simd::float3 c7 = create_centroid(i7, n, 4.0, n2, 2.0);
                /*
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
                 */
                simd::float3 points[8] = {c0, c1, c2, c3, c4, c5, c6, c7};
                float values[8] = {
                    fabs(voxels[i0]),
                    fabs(voxels[i1]),
                    fabs(voxels[i2]),
                    fabs(voxels[i3]),
                    fabs(voxels[i4]),
                    fabs(voxels[i5]),
                    fabs(voxels[i6]),
                    fabs(voxels[i7])
                    /*
                    (voxels[i0]),
                    (voxels[i1]),
                    (voxels[i2]),
                    (voxels[i3]),
                    (voxels[i4]),
                    (voxels[i5]),
                    (voxels[i6]),
                    (voxels[i7])
                     */
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
                             //const void* centroids,
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
    //float diag = 2.0 * sqrt(3.0f) * (resolution[0] / dimension);
    //int count = width * height;
    //int size = pow(dimension, 3.0);
    int square = pow(dimension, 2.0);
    float offset = resolution[0] * 0.5;
    
    // Relative camera variables
    simd_float3x3 K = ((simd_float3x3 *) intrisics)[0];
    simd_float4x4 Rt = ((simd_float4x4 *) camera_pose)[0];
    simd_float3x3 Kinv = simd_inverse(K);
    simd_float4x4 tRt = simd_transpose(Rt);
    simd_float4x3 Rtc = simd_matrix_from_rows(tRt.columns[0], tRt.columns[1], tRt.columns[2]);
    simd_float3x3 rotation = simd_matrix(Rtc.columns[0], Rtc.columns[1], Rtc.columns[2]);
    simd_float3 translation = simd_make_float3(Rtc.columns[3][0],Rtc.columns[3][1],Rtc.columns[3][2]);
    //simd_float3 resolve = simd_make_float3(resolution[0], resolution[1], resolution[2]);
    
    // Determines bounding box of camera, O(n) where n is width*height of depthmap.
    // It can reduce (always ?) next loop complexity.
    simd_float2x3 box = cameraBoxing(depthmap, width, height, rotation, translation, Kinv);
    simd_int3 point_min = simd_make_int3(global_to_integer(box.columns[0].x + offset, dimension, resolution[0]),
                                         global_to_integer(box.columns[0].y + offset, dimension, resolution[1]),
                                         global_to_integer(box.columns[0].z, dimension, resolution[2]));
    simd_int3 point_max = simd_make_int3(global_to_integer(box.columns[1].x + offset, dimension, resolution[0]),
                                         global_to_integer(box.columns[1].y + offset, dimension, resolution[1]),
                                         global_to_integer(box.columns[1].z, dimension, resolution[2]));
    int mini = hash_function(point_min, dimension);
    int maxi = hash_function(point_max, dimension);

    for( int i = mini; i<maxi; i++) {
        //simd::float3 centroid = ((simd::float3 *) centroids)[i];
        simd::float3 centroid = create_centroid(i,
                                                dimension,
                                                resolution[0],
                                                square,
                                                offset);
        simd::float3 local = simd_mul(rotation, centroid + translation);
        
        simd::float3 temp = simd_mul(K, local / local.z);
        simd::float3 project = simd_make_float3(temp.x, temp.y, local.z);
        int u = (int) project.x;
        int v = (int) project.y;
        if (local.z < 0)            continue;
        if (u < 0 || u >= height)   continue;
        if (v < 0 || v >= width)    continue;
        
        float z = local.z; //simd_length(local) ou local.z
        float zp = depthmap[u * width + v];
        
        // Depth invalid
        if (zp < 0.000001) continue;
        float distance = zp - z;
        if (distance >= delta + epsilon && distance <= zp) carving_voxel((Voxel *)voxels, i);
        if (fabs(distance) < delta) update_voxel((Voxel *)voxels, distance, 1, i);
        //else if (distance > delta) update_voxel((Voxel *)voxels, delta, 1, i);
        //else update_voxel((Voxel *)voxels, -delta, 1, i);
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
