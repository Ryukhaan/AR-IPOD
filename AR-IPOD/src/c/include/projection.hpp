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
    //int square = dimension * dimension;
    //int size = pow(dimension, 3.0);
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
    simd_float2x3 box = compute_bounding_box(depthmap, width, height, R, T, Kinv);
    simd_int3 point_min = global_to_integer(box.columns[0] + offset, resolution);
    simd_int3 point_max = global_to_integer(box.columns[1] + offset, resolution);
    int mini = hash_code(point_min, dimension);
    int maxi = hash_code(point_max, dimension);
    //for (int i = mini; i<maxi; i++) {
        //if (i >= size || i < 0) continue;
    /*
    for (int i = 0; i<width*height; i++) {
        float depth = depthmap[i];
        if (depth == 0) continue;
        int u = i / width;
        int v = i % width;
        simd::float4 homogene = simd_make_float4(u * depth, v * depth, depth, 1);
        simd::float4 local = simd_mul(Kinv, homogene);
        simd::float3 rlocal = simd_make_float3(local.x, local.y, local.z);
        simd::float3 rglobal = simd_mul(R, rlocal + T);
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
    */

    //for (int i = 0; i<size; i++)
    //for (int i = mini; i<maxi; i++)
    int n = -1;
    float global_offset = 0.5 * (1.0 - dimension) * resolution;
    for (int i = 0; i<dimension; i++)
        for (int j = 0; j<dimension; j++)
            for (int k = 0; k<dimension; k++)
    {
        //if (i < 0 || i >= size) continue;
        n ++;
        if (n <= mini || n >= maxi) continue;
        simd_int3   v_ijk = simd_make_int3(i, j, k);
        //simd::float3 centroid = create_centroid(n, resolutions.x, dimension) - offset;
        //simd::float3 centroid = (integer_to_global(v_ijk, resolution) + (resolution * 0.5)) - offset;
        simd::float3 centroid = integer_to_global(v_ijk, resolution) + global_offset;
        //simd::float3 X_L    = simd_mul(R, centroid + T);
        
        // Project World to Camera
        simd::float3 X_L      = simd_mul(simd_transpose(R), centroid - T);
        simd::float4 homogene = simd_make_float4(X_L.x, X_L.y, X_L.z, 1);
        // Behind camera
        if (X_L.z < 0) continue;
            
        // Project Camera to Image
        simd::float4 project  = simd_mul(K, homogene);
        int u = (int) (project.x / project.z);
        int v = (int) (project.y / project.z);
        // Depthmap OOB
        if (u < 0 || u >= height) continue;
        if (v < 0 || v >= width) continue;
        
        // Depthmap value
        float z = depthmap[u * width + v];
        // Invalid depth
        if (std::isnan(z)) continue;
        if (z < 1e-6) continue;
        
        simd::float4 uvz = simd_make_float4(z * u, z * v, z, 1.0);
        simd::float4 X_S = simd_mul(Kinv, uvz);
        // Depth invalid
        float distance = simd_sign(z - project.z) * simd_distance(X_S, homogene);
        //float distance = z - project.z;
        
        //weight = distance > delta ? 0.0 : distance <= epsilon ? 1.0 : (distance - delta) / (epsilon - delta);
        float weight = weighting(distance, delta, epsilon);
        if (weight == 0.0) continue;
        
        if (distance >= delta + epsilon)
            //carving_voxel_at((Voxel *)voxels, n);
            ((Voxel *)voxels)[n] = carving_voxel_at(((Voxel *)voxels)[n]);
        if (fabs(distance) < delta)
            //update_voxel_at((Voxel *)voxels, distance, weight, n);
            ((Voxel *)voxels)[n] = update_voxel_at(((Voxel *)voxels)[n], distance, weight);
        /*
        if (distance < -delta) {
            ((Voxel *)voxels)[n] = update_voxel_at(((Voxel *)voxels)[n], -delta, weight);
        }
        else {
            ((Voxel *)voxels)[n] = update_voxel_at(((Voxel *)voxels)[n], distance, weight);
        }
        */
        //else if (distance < -delta)
        //    update_voxel((Voxel *)voxels, -delta, weight, i);
        //else if (distance > delta)
        //    update_voxel((Voxel *)voxels, delta, weight, i);
    }
    return number_of_changes;
}
#endif /* projection_hpp */