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
#include <iostream>
#include <limits>
#include <fstream>
//#include <omp.h>

#include "linalg.hpp"
#include "marching_cube.hpp"
#include "io.hpp"
#include "tsdf.hpp"
#include "filtering.hpp"
//#include "raytracing.hpp"
//#include "icp.hpp"
#include "projection.hpp"

using namespace std;
                                          
/*
 * GLOBAL FUNCTION (hpp)
 */

unsigned long bridge_extractMesh(void* triangles,
                                 void* voxels,
                                 const int edgeTable[256],
                                 const int triTable[4096],
                                 const int dimension,
                                 const float isolevel,
                                 const float resolution) {
    unsigned long index = 0;
    const int n = dimension;
    const float offset = 0.5 * (dimension * resolution);
    int n2 = n * n;
    //float cy = 1, cx = 1;
    //float cy = 6, cx = 6;
//  int num_threads = omp_get_num_threads();
//#pragma omp parallel for shared(index) collapse(3) num_threads(num_threads)
    for (int i = 0; i<n-1; i++) {
        for (int j = 0; j<n-1; j++) {
            for (int k = 0; k<n-1; k++) {
                int idx = i * n2 + j * n + k;
                
                int i0 = idx;
                int i1 = idx + n2;
                int i2 = idx + n2 + 1;
                int i3 = idx + 1;
                int i4 = idx + n;
                int i5 = idx + n + n2;
                int i6 = idx + n + n2 + 1;
                int i7 = idx + n + 1;
                
                simd_int3 p0 = simd_make_int3(i, j, k);
                simd_int3 p1 = simd_make_int3(i+1, j, k);
                simd_int3 p2 = simd_make_int3(i+1, j, k+1);
                simd_int3 p3 = simd_make_int3(i, j, k+1);
                simd_int3 p4 = simd_make_int3(i, j+1, k);
                simd_int3 p5 = simd_make_int3(i+1, j+1, k);
                simd_int3 p6 = simd_make_int3(i+1, j+1, k+1);
                simd_int3 p7 = simd_make_int3(i, j+1, k+1);
                
                simd::float3 c0 = integer_to_global(p0, resolution) - offset;
                simd::float3 c1 = integer_to_global(p1, resolution) - offset;
                simd::float3 c2 = integer_to_global(p2, resolution) - offset;
                simd::float3 c3 = integer_to_global(p3, resolution) - offset;
                simd::float3 c4 = integer_to_global(p4, resolution) - offset;
                simd::float3 c5 = integer_to_global(p5, resolution) - offset;
                simd::float3 c6 = integer_to_global(p6, resolution) - offset;
                simd::float3 c7 = integer_to_global(p7, resolution) - offset;

                simd::float3 points[8] = {c0, c1, c2, c3, c4, c5, c6, c7};
                float values[8] = {
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
                //std::vector<simd::float3> temp = polygonise_dmc(points, values, isolevel);
                if (temp.size() <= 0) continue;
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

void bridge_integrateDepthMap(float* depthmap,
                              //const void* centroids,
                              const void* rotation,
                              const void* translation,
                              const void* intrinsics,
                              void* voxels,
                              const int width,
                              const int height,
                              const int dimension,
                              const float resolution,
                              const float delta,
                              const float epsilon,
                              const float lambda,
                              const float cx,
                              const float cy) {
    integrate_projection(depthmap,
                         rotation, translation, intrinsics,
                         voxels,
                         width, height, dimension, resolution,
                         delta, epsilon, lambda, cx, cy);
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

/*
void bridge_fast_icp(const float* previous_points,
                     const float* current_points,
                     const void* intrinsics,
                     void* rotation,
                     void* lie_rotation,
                     void* translation,
                     void* voxels,
                     const int dimension,
                     const float resolution,
                     const int width,
                     const int height) {
    fast_icp(previous_points, current_points, intrinsics, rotation, lie_rotation, translation, voxels, dimension, resolution, width, height);
}
*/
/*
void bridge_global_registration(const float* previous,
                                const float* current,
                                //const void* voxels,
                                const int width,
                                const int height,
                                void* rotation,
                                void* translation,
                                const void* intrinsics,
                                const float resolution,
                                const int dimension,
                                const float thresh_depth,
                                const float corresp_dist,
                                const int max_num_iter)
{
    //super4PCS(previous, current, voxels, width, height, rotation, translation, intrinsics, resolution, dimension);
    //icp(previous, current, width, height, rotation, translation, intrinsics, resolution, dimension, thresh_depth, corresp_dist, max_num_iter);
 
}
*/

void bridge_median_filter(float* depthmap,
                          const int window_size,
                          const int width,
                          const int height) {
    median_filter(depthmap, window_size, width, height);
}

void bridge_drift_correction(const float* current_points,
                             const void* intrinsics,
                             void* extrinsics,
                             const void* voxels,
                             const int dimension,
                             const float resolution,
                             const int width,
                             const int height) {
    simd_float3x3 K     = ((simd_float3x3 *) intrinsics)[0];
    simd_float4x3 Rt    = ((simd_float4x3 *) extrinsics)[0];
    simd_float3x3 Kinv  = simd_inverse(K);
    simd_float3x3 rotation  = simd_matrix(Rt.columns[0], Rt.columns[1], Rt.columns[2]);
    simd_float3 translation = Rt.columns[3];
    float offset    = (dimension * resolution) * 0.5;
    int square      = pow(dimension, 2.0);
    int size        = pow(dimension, 3.0);
    simd_float4x4 xtilde = simd_diagonal_matrix(simd_make_float4(0, 0, 0, 0));
    simd_float3x4 xy;
    simd::float3 resolutions = simd_make_float3(resolution, resolution, resolution);
    for (int i=0; i<height; i++)
    {
        for (int j = 0; j<width; j++)
        {
            float depth = current_points[ i*width + j];
            if ( ! std::isnan(depth) && depth > 1e-6)
            {
                simd::float3 uv = simd_make_float3(i, j, 1);
                simd::float3 global  = simd_mul(simd_transpose(rotation), simd_mul(Kinv, depth * uv) - translation);
                simd_int3 global_int = global_to_integer(global + offset, resolutions);
                int k = hash_code(global_int, dimension);
                if (k >= size || k < 0) continue;
                float phi_p = ((Voxel *) voxels)[k].sdf;
                simd_float3 M = simd_make_float3(0, 0, 0);
                M[0] = ((Voxel *) voxels)[k + 1].sdf - phi_p;
                M[1] = ((Voxel *) voxels)[k + dimension].sdf - phi_p;
                M[2] = ((Voxel *) voxels)[k + square].sdf - phi_p;
                if (simd_length(M) == 0) continue;
                M = (- M / simd_length(M)) * phi_p;
                for (int n = 0; n<4; n++)
                    for (int m = 0; m<4; m++)
                        xtilde.columns[m][n] += global[n] * global[m];
                for (int n = 0; n<4; n++)
                    for (int m=0; m<3; m++)
                        xy.columns[m][n] += global[n] * M[m];
            }
        }
    }
    //xtilde = simd_inverse(xtilde);
    //simd_float4x3 error = simd_transpose(simd_mul(xtilde, xy));
}
/*
void bridge_raycastDepthMap(float* depthmap,
                           const void* rotation,
                           const void* translation,
                           const void* intrinsics,
                           void* voxels,
                           const int width,
                           const int height,
                           const int dimension,
                           const float resolution,
                           const float delta,
                           const float epsilon,
                           const float lambda) {
    simd_float4x4 K = ((simd_float4x4 *) intrinsics)[0];
    simd_float3x3 R = ((simd_float3x3 *) rotation)[0];
    simd_float3   T = ((simd_float3 *) translation)[0];
    simd_float4x4 Kinv = simd_inverse(K);
    Voxel *p_voxels = (Voxel *) voxels;
    raytracing(depthmap, R, T, Kinv, p_voxels, width, height, dimension, resolution, delta, epsilon, lambda);
}
*/
