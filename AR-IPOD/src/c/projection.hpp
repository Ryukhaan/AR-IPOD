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

int integrate_projection(float* depthmap,
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
                         const float lambda)
{
    // Instanciate all local variables
    int number_of_changes = 0;
    //float diag = 2.0 * sqrt(3.0f) * (resolution[0] / dimension);
    //int count = width * height;
    int size = pow(dimension, 3.0);
    simd::float3 resolutions = simd_make_float3(resolution, resolution, resolution);
    simd::float3 offset = 0.5 * (dimension * resolutions);
    
    // Relative camera variables
    simd_float4x4 K = ((simd_float4x4 *) intrinsics)[0];
    simd_float3x3 R = ((simd_float3x3 *) rotation)[0];
    simd_float3   T = ((simd_float3 *) translation)[0];
    simd_float4x4 Kinv  = simd_inverse(K);
    
    //simd::float3 ot     = simd_make_float3(Rt.columns[3].x, Rt.columns[3].y, Rt.columns[3].z);
    // Determines bounding box of camera, O(n) where n is width*height of depthmap.
    // It can reduce (always ?) next loop complexity.
    /*
    simd_float2x3 box = compute_bounding_box(depthmap, width, height, R, T, Kinv);
    simd_int3 point_min = global_to_integer(box.columns[0] + offset, resolution);
    simd_int3 point_max = global_to_integer(box.columns[1] + offset, resolution);
    int mini = hash_code(point_min, dimension);
    int maxi = hash_code(point_max, dimension);
    */
    //for (int i = mini; i<maxi; i++) {
        //if (i >= size || i < 0) continue;
    for (int i = 0; i<width*height; i++) {
        float depth = depthmap[i];
        if (depth == 0) continue;
        int u = i / width;
        int v = i % width;
        simd::float4 homogene = simd_make_float4(u * depth, v * depth, depth, 1);
        simd::float4 local = simd_mul(Kinv, homogene);
        simd::float3 rlocal = simd_make_float3(local.x, local.y, local.z);
        simd::float3 rglobal = simd_mul(simd_transpose(R), rlocal - T);
        //simd::float3 rglobal = rlocal;
        if (! (rglobal.x >= -offset.x && rglobal.y >= -offset.y && rglobal.z >= -offset.z &&
               rglobal.x < offset.x && rglobal.y < offset.x && rglobal.z < offset.z))
            continue;
        simd_int3 coordinate = global_to_integer(rglobal + offset, resolution);
        int k = hash_code(coordinate, dimension);
        if (k < 0 || k >= size) continue;
        update_voxel((Voxel *)voxels, -0.1, 1, k);
        number_of_changes ++;
    }
    /*
    for (int i = 0; i<size; i++)
    //for (int i = mini; i<maxi; i++)
    {
        //if (i < 0 || i >= size) continue;
        simd::float3 centroid = create_centroid(i, resolutions.x, dimension) - offset;
        //simd::float3 X_L      = simd_mul(R, centroid + T);
        simd::float3 X_L      = simd_mul(simd_transpose(R), centroid - T);
        simd::float4 homogene = simd_make_float4(X_L.x, X_L.y, X_L.z, 1);
        simd::float4 project  = simd_mul(K, homogene);
        
        int u = (int) (project.x / project.z);
        int v = (int) (project.y / project.z);
        if (u < 0 || u >= height) continue;
        if (v < 0 || v >= width) continue;
        
        float z = depthmap[u * width + v];
        simd::float4 uvz = simd_make_float4(z * u, z * v, z, 1.0);
        simd::float4 X_S = simd_mul(Kinv, uvz);
        // Depth invalid
        if (std::isnan(z)) continue;
        if (z < 1e-6) continue;
        float distance = simd_distance(X_S, homogene);
        //float distance = z - project.z;
        distance = simd_sign(z - project.z) * distance;
        
        float weight = constant_weighting();
        //float weight = weighting(distance, delta, epsilon);
        
        //if (distance >= delta + epsilon)
        //    carving_voxel((Voxel *)voxels, i);
        if (fabs(distance) <= delta)
            update_voxel((Voxel *)voxels, distance, weight, i);
        //else if (distance < -delta)
        //    update_voxel((Voxel *)voxels, -delta, weight, i);
    }
    */
    return number_of_changes;
}
#endif /* projection_hpp */
