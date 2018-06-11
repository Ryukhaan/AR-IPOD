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
     simd_float2x3 box = compute_bounding_box(depthmap, width, height, Rtinv, Kinv);
     simd_int3 point_min = global_to_integer(box.columns[0] + offset, resolution);
     simd_int3 point_max = global_to_integer(box.columns[1] + offset, resolution);
     int mini = hash_code(point_min, dimension);
     int maxi = hash_code(point_max, dimension);
    */
    /*
    for (int i = 0; i<width*height; i++) {
        //for (int i = mini; i<maxi; i++) {
        //if (i >= size || i < 0) continue;
        float depth = depthmap[i];
        int u = i / width;
        int v = i % width;
        simd::float4 homogene = simd_make_float4(u * depth, v * depth, depth, 1);
        simd::float4 local = simd_mul(Kinv, homogene);
        simd::float3 rlocal = simd_make_float3(local.x, local.y, local.z);
        simd::float3 rglobal = simd_mul(simd_transpose(R), rlocal - T);
        if (! (rglobal.x >= -offset.x && rglobal.y >= -offset.y && rglobal.z >= -offset.z &&
               rglobal.x < offset.x && rglobal.y < offset.x && rglobal.z < offset.z))
            continue;
        simd_int3 coordinate = global_to_integer(rglobal + offset, resolution);
        int k = hash_code(coordinate, dimension);
        if (k < 0 || k >= size) continue;
        update_voxel((Voxel *)voxels, -0.1, 1, k);
    }
    */
    
    //for( int i = mini; i<maxi; i++)
    /*
     for (int i = 0; i<size; i++)
     {
     //simd::float3 centroid = create_centroid(i, resolution, dimensions, square, offset);
     simd::float3 centroid = create_centroid(i, resolutions.x, dimension) - offset;
     
     simd::float4 homogene_3d = simd_make_float4(centroid.x, centroid.y, centroid.z, 1);
     float z = simd_distance(ot, centroid);
     //simd::float3 local = simd_mul(rotation, centroid + translation);
     //simd::float4 local = simd_mul(Rt, homogene_3d);
     //simd::float3 local = simd_mul(simd_transpose(rotation), centroid-translation);
     
     //simd::float4 temp = simd_mul(K, local);
     simd::float4 project = simd_mul(simd_mul(K, Rt), homogene_3d);
     int u = (int) (project.x / project.z); //(unhomogene_2d.x / unhomogene_2d.z);
     int v = (int) (project.y / project.z); //(unhomogene_2d.y / unhomogene_2d.z);
     //if (local.z < 0)            continue;
     if (u < 0 || u >= height)   continue;
     if (v < 0 || v >= width)    continue;
     float zp = depthmap[u * width + v];
     
     // Depth invalid
     if (std::isnan(zp)) continue;
     //if (zp > 1.0) continue;
     
     float distance = zp - z;
     // Calculate weight
     float weight = constant_weighting();
     
     if (fabs(distance) < delta) update_voxel((Voxel *)voxels, distance, weight, i);
     else if (distance >= delta + epsilon)  carving_voxel((Voxel *)voxels, i);
     //else if (distance > delta) update_voxel((Voxel *)voxels, delta, 1, i);
     //else update_voxel((Voxel *)voxels, -delta, 1, i);
     }
     */
    for (int i = 0; i<size; i++)
    //for (int i = mini; i<maxi; i++)
    {
        if (i < 0 || i >= size) continue;
        simd::float3 centroid = create_centroid(i, resolutions.x, dimension) - offset;
        simd::float3 X_L      = simd_mul(R, centroid + T);
        simd::float4 homogene = simd_make_float4(X_L.x, X_L.y, X_L.z, 1);
        simd::float4 project  = simd_mul(K, homogene);
        if (X_L.z < 1e-6) continue;
        
        int u = (int) (project.x / project.z); //(unhomogene_2d.x / unhomogene_2d.z);
        int v = (int) (project.y / project.z); //(unhomogene_2d.y / unhomogene_2d.z);
        //if (local.z < 0)            continue;
        if (u < 0 || u >= height)   continue;
        if (v < 0 || v >= width)    continue;
        
        float z = depthmap[u * width + v];
        // Depth invalid
        if (std::isnan(z)) continue;
        
        //simd::float4 X      = simd_make_float4(u * z, v * z, z, 1);
        //simd::float4 X_S    = simd_mul(Kinv, X);
        //float distance = simd_distance(X_S, X_L);
        float distance = X_L.z - z;
        
        //float weight = weighting(distance, delta, epsilon);
        float weight = constant_weighting();
        //distance = fabs(distance) <= delta ? distance : distance > delta ? delta : -delta;
        //update_voxel((Voxel *)voxels, distance, weight, i);
        if (distance >= delta + epsilon)  carving_voxel((Voxel *)voxels, i);
        if (fabs(distance) <= delta) update_voxel((Voxel *)voxels, distance, weight, i);
        //else if (distance >= delta + epsilon)  carving_voxel((Voxel *)voxels, i);
        //else if (fabs(distance) >= delta + epsilon && distance <= X_S.z) carving_voxel((Voxel *)voxels, i);
        //else if (distance >= delta && distance < delta + epsilon) update_voxel((Voxel *)voxels, delta, weight, i);
        //else if (distance < -delta) update_voxel((Voxel *)voxels, -delta, weight, i);
        //else if (distance > delta) update_voxel((Voxel *) voxels, delta, weight, i);
    }
    return number_of_changes;
}
#endif /* projection_hpp */
