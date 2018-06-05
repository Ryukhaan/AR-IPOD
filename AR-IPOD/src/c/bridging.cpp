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
#include "filtering.hpp"

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
        ((simd::float3*) centroids)[i][2] = integer_to_global(z, size, resolution) - offset;
    }
}

unsigned long bridge_extractMesh(void* triangles,
                                 void* voxels,
                                 //const float* voxels,
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
                    /*
                    fabs( ((Voxel*) voxels)[i0].sdf ),
                    fabs( ((Voxel*) voxels)[i1].sdf ),
                    fabs( ((Voxel*) voxels)[i2].sdf ),
                    fabs( ((Voxel*) voxels)[i3].sdf ),
                    fabs( ((Voxel*) voxels)[i4].sdf ),
                    fabs( ((Voxel*) voxels)[i5].sdf ),
                    fabs( ((Voxel*) voxels)[i6].sdf ),
                    fabs( ((Voxel*) voxels)[i7].sdf )
                    */
                    ( ((Voxel*) voxels)[i0].sdf ),
                    ( ((Voxel*) voxels)[i1].sdf ),
                    ( ((Voxel*) voxels)[i2].sdf ),
                    ( ((Voxel*) voxels)[i3].sdf ),
                    ( ((Voxel*) voxels)[i4].sdf ),
                    ( ((Voxel*) voxels)[i5].sdf ),
                    ( ((Voxel*) voxels)[i6].sdf ),
                    ( ((Voxel*) voxels)[i7].sdf )
                };
                std::vector<simd::float3> temp = polygonise(points, values, isolevel, edgeTable, triTable);
                if (temp.size() == 0) continue;
                /*
                simd::float3* more_triangles = nullptr;
                more_triangles = (simd::float3*) realloc(triangles, (index + temp.size()) * sizeof(simd::float3));
                if ( more_triangles != nullptr )
                    triangles = more_triangles;
                else {
                    free(triangles);
                    return index;
                }
                //triangles = (simd::float3*) realloc(triangles, );
                */
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

int bridge_integrateDepthMap(float* depthmap,
                             //const void* centroids,
                             const void* extrinsics,
                             const void* intrinsics,
                             void* voxels,
                             const int width,
                             const int height,
                             const int dimension,
                             const float resolution[3],
                             const float delta,
                             const float epsilon,
                             const float lambda) {
    //median_filter(depthmap, 2, width, height);
    // Instanciate all local variables
    int number_of_changes = 0;
    //float diag = 2.0 * sqrt(3.0f) * (resolution[0] / dimension);
    //int count = width * height;
    int size = pow(dimension, 3.0);
    int square = pow(dimension, 2.0);
    float offset = resolution[0] * 0.5;
    
    // Relative camera variables
    simd_float3x3 K = ((simd_float3x3 *) intrinsics)[0];
    simd_float4x3 Rt = ((simd_float4x3 *) extrinsics)[0];
    simd_float3x3 Kinv = simd_inverse(K);
    simd_float3x3 rotation = simd_matrix(Rt.columns[0], Rt.columns[1], Rt.columns[2]);
    simd_float3 translation = Rt.columns[3];
    //simd_float3 resolve = simd_make_float3(resolution[0], resolution[1], resolution[2]);
    
    // Determines bounding box of camera, O(n) where n is width*height of depthmap.
    // It can reduce (always ?) next loop complexity.
    /*
    simd_float2x3 box = bounding_box_and_filter(depthmap, width, height, rotation, translation, Kinv, 2);
    simd_int3 point_min = simd_make_int3(global_to_integer(box.columns[0].x + offset, dimension, resolution[0]),
                                         global_to_integer(box.columns[0].y + offset, dimension, resolution[1]),
                                         global_to_integer(box.columns[0].z + offset, dimension, resolution[2]));
    simd_int3 point_max = simd_make_int3(global_to_integer(box.columns[1].x + offset, dimension, resolution[0]),
                                         global_to_integer(box.columns[1].y + offset, dimension, resolution[1]),
                                         global_to_integer(box.columns[1].z + offset, dimension, resolution[2]));
    int mini = hash_function(point_min, dimension);
    int maxi = hash_function(point_max, dimension);
    */
    //for( int i = mini; i<maxi; i++) {
    //    if (i >= size) continue;
    //    if (i < 0) continue;
    for (int i = 0; i<size; i++) {
        //simd::float3 centroid = ((simd::float3 *) centroids)[i];
        simd::float3 centroid = create_centroid(i,
                                                dimension,
                                                resolution[0],
                                                square,
                                                offset);
        simd::float3 local = simd_mul(rotation, centroid + translation);
        //simd::float3 local = simd_mul(simd_transpose(rotation), centroid-translation);
        
        simd::float3 temp = simd_mul(K, local / local.z);
        simd::float3 project = simd_make_float3(temp.x, temp.y, local.z);
        int u = (int) project.x;
        int v = (int) project.y;
        //if (local.z < 0)            continue;
        if (u < 0 || u >= height)   continue;
        if (v < 0 || v >= width)    continue;
        
        float z = local.z; //simd_length(local) ou local.z
        float zp = depthmap[u * width + v];
        // Depth invalid
        if (std::isnan(zp)) continue;
        //if (zp > 1.0) continue;
        
        float distance = zp - z;
        // Calculate weight
        float w = constant_weighting(distance, delta, lambda);
        
        if (distance >= delta + epsilon && distance <= zp) carving_voxel((Voxel *)voxels, i);
        if (fabs(distance) < delta) update_voxel((Voxel *)voxels, distance, w, i);
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

void bridge_fast_icp(const float* last_points,
                     const float* current_points,
                     const void* intrinsics,
                     void* extrinsics,
                     const int width,
                     const int height) {
    simd_float3x3 K = ((simd_float3x3 *) intrinsics)[0];
    simd_float4x3 Rt = ((simd_float4x3 *) extrinsics)[0];
    simd_float3x3 Kinv = simd_inverse(K);
    simd_float3x3 rotation = simd_matrix(Rt.columns[0], Rt.columns[1], Rt.columns[2]);
    simd_float3 translation = Rt.columns[3];
    //simd_float3 translation = simd_make_float3(0, 0, 0);
    
    //for (int k=0; k<4; k++) {
    int count_last = 0;
    int count_current = 0;
    simd::float3 centre_last = simd_make_float3(0, 0, 0);
    simd::float3 centre_current = simd_make_float3(0, 0, 0);
    for (int i=0; i<height; i++) {
        for (int j = 0; j<width; j++) {
            // Unproject point
            if ( ! std::isnan(last_points[ i*width + j]) && last_points[ i*width + j] > 1e-6) {
                float depth = last_points[ i*width + j];
                simd::float3 uv = simd_make_float3(i, j, 1);
                simd::float3 local_point = simd_mul(simd_transpose(rotation), simd_mul(Kinv, depth * uv) - translation);
                count_last ++;
                centre_last = centre_last + local_point;
            }
            if ( ! std::isnan(current_points[ i*width + j]) && current_points[ i*width + j] > 1e-6) {
                float depth = current_points[ i*width + j];
                simd::float3 uv = simd_make_float3(i, j, 1);
                simd::float3 local_point = simd_mul(simd_transpose(rotation), simd_mul(Kinv, depth * uv) - translation);
                count_current ++;
                centre_current = centre_current + local_point;
            }
        }
    }
    centre_current = centre_current / (float) count_current;
    centre_last = centre_last / (float) count_last;
    //simd::float3 tmp = translation;
    translation += centre_last - centre_current;
    //}
    ((simd_float4x3 *) extrinsics)[0].columns[3] = translation;
}
