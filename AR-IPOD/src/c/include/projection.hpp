//
//  projection.hpp
//  AR-IPOD
//
//  Created by Remi Decelle on 11/06/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#ifndef projection_hpp
#define projection_hpp

#include <stdio.h>
#include <simd/common.h>
#include <simd/vector_types.h>
#include <simd/vector_make.h>
#include <simd/vector.h>
#include <simd/conversion.h>
#include <simd/matrix.h>

#include "constants.h"

void integrate_projection(float* depthmap,
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
                          const float cy)
{
    // Instanciate all local variables
    simd::float3 resolutions = simd_make_float3(resolution, resolution, resolution);
    simd::float3 offset = 0.5 * (dimension * resolutions);
    
    // Relative camera variables
    simd_float4x4 K = ((simd_float4x4 *) intrinsics)[0];
    simd_float3x3 R = ((simd_float3x3 *) rotation)[0];
    simd_float3   T = ((simd_float3 *) translation)[0];
    simd_float4x4 Kinv  = simd_inverse(K);
    simd_float4x4 Rt;
    for (int i = 0; i<3; i++) Rt.columns[i] = simd_make_float4(R.columns[i].x, R.columns[i].y, R.columns[i].z, 0);
    Rt.columns[3] = simd_make_float4(T.x, T.y, T.z, 1);
    
    // Determines bounding box of camera, O(n) where n is width*height of depthmap.
    // It can reduce (always ?) next loop complexity.
    
    simd_float2x3 box = compute_bounding_box(depthmap, width, height, R, T, Kinv, cx, cy);
    simd_int3 point_min = global_to_integer(box.columns[0] + offset, resolution);
    simd_int3 point_max = global_to_integer(box.columns[1] + offset, resolution);
    int mini = hash_code(point_min, dimension);
    int maxi = hash_code(point_max, dimension);
    
    /*
    for (int i = 0; i<width*height; i++) {
        float depth = depthmap[i];
        if (depth == 0) continue;
        int u = i / width;
        int v = i % width;
        simd::float4 homogene = simd_make_float4(u * depth * cx, v * depth * cy, depth, 1);
        simd::float4 local = simd_mul(Kinv, homogene);
        simd::float3 rlocal = simd_make_float3(local.x, local.y, local.z);
        simd::float3 rglobal = simd_mul(R, rlocal) + T;
        //simd::float3 rglobal = simd_mul(simd_transpose(R), rlocal - T);
        //simd::float3 rglobal = rlocal;
        if (! (rglobal.x >= -offset.x && rglobal.y >= -offset.y && rglobal.z >= -offset.z &&
               rglobal.x < offset.x && rglobal.y < offset.x && rglobal.z < offset.z))
            continue;
        simd_int3 coordinate = global_to_integer(rglobal + offset, resolution);
        int k = hash_code(coordinate, dimension);
        if (k < 0 || k >= 16777216) continue;
        Voxel updated_voxel = ((Voxel *)voxels)[k];
        updated_voxel = update_voxel(updated_voxel, -0.1, 1);
        ((Voxel *)voxels)[k] = updated_voxel;
        //number_of_changes ++;
    }
    */
    //for (int i = 0; i<size; i++)
    //for (int i = mini; i<maxi; i++)
    int n = -1;
    //float global_offset = 0.5 * (1.0 - dimension) * resolution;
    
    float global_offset = 0.5 * dimension * resolution;
    for (int i = 0; i<dimension; i++)
        for (int j = 0; j<dimension; j++)
            for (int k = 0; k<dimension; k++)
            {
                //if (i < 0 || i >= size) continue;
                n ++;
                if (n <= mini || n >= maxi) continue;
                simd_int3   v_ijk = simd_make_int3(i, j, k);
                //simd::float3 centroid = create_centroid(n, resolutions.x, dimension) - offset;
                simd::float3 centroid = integer_to_global(v_ijk, resolution) - global_offset;
                //simd::float4 higher = simd_make_float4(centroid.x, centroid.y, centroid.z, 1);
                //float z = simd_distance(centroid, T);
                
                simd::float3 X_L      = simd_mul(simd_transpose(R), centroid - T);
                //simd::float3 X_L      = simd_mul(R, centroid) + T;
                //simd::float3 X_L        = global_to_local(centroid, R, T);
                if (X_L.z < 0) continue;
                
                simd::float4 homogeneous = simd_make_float4(X_L.x, X_L.y, X_L.z, 1);
                simd::float4 project  = simd_mul(K, homogeneous);
                //simd::float4 project    = unproject_local_to_screen(homogeneous, K);
                
                int u = (int) (project.x / (project.z  * cx));
                int v = (int) (project.y / (project.z  * cy));
                if (u < 0 || u >= height) continue;
                if (v < 0 || v >= width) continue;
                
                float zp = depthmap[u * width + v];
                if (std::isnan(zp)) continue;
                if (zp < 1e-7) continue;
                
                simd::float4 uvz = simd_make_float4(zp * u * cx, zp * v * cy, zp, 1.0);
                //simd::float4 uvz = homogenezie(u, v, zp, cx, cy);
                simd::float4 X_S = simd_mul(Kinv, uvz);
                //simd::float4 X_S = project_screen_to_local(uvz, Kinv);
                // Depth invalid
                
                float distance = simd_sign(zp - X_L.z) * simd_distance(X_S , homogeneous);
                //float distance = zp - X_L.z;
                //float distance = zp - z;
                
                float weight = 1.0;
                //float weight = weighting(distance, delta, epsilon);
                
                //
                // if (distance < -delta)
                // update_voxel((Voxel *)voxels, -delta, weight, n);
                // else if (distance > delta)
                // update_voxel((Voxel *)voxels, delta, weight, n);
                // else
                // update_voxel((Voxel *)voxels, distance, weight, n);
                //
                Voxel updated_voxel = ((Voxel *)voxels)[n];
               // if (distance >= delta + epsilon && distance < zp)
               //     updated_voxel = carving_voxel(updated_voxel);
                if (fabs(distance) <= delta)
                    updated_voxel = update_voxel(updated_voxel, distance, weight);
                else if (distance < -delta)
                    updated_voxel = update_voxel(updated_voxel, -delta, weight);
                else if (distance > delta)
                    updated_voxel = update_voxel(updated_voxel, delta, weight);
                ((Voxel *)voxels)[n] = updated_voxel;
            }

    return;
}
#endif /* projection_hpp */
