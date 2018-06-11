//
//  icp.hpp
//  AR-IPOD
//
//  Created by Remi Decelle on 18/05/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#ifndef icp_hpp
#define icp_hpp

#include <stdio.h>
#include <simd/common.h>
#include <simd/vector_types.h>
#include <simd/vector_make.h>
#include <simd/vector.h>
#include <simd/conversion.h>
#include <simd/matrix.h>

void fast_icp(const float* previous_points,
              const float* current_points,
              const void* intrinsics,
              void* rotation,
              void* translation,
              void* voxels,
              const int dimension,
              const float resolution,
              const int width,
              const int height)
{
    simd_float4x4 K = ((simd_float4x4 *) intrinsics)[0];
    simd_float3x3 R = ((simd_float3x3 *) rotation)[0];
    simd::float3  T = ((simd_float3 *) translation)[0];
    simd_float4x4 Kinv = simd_inverse(K);
    //simd::float3 dimensions = simd_make_float3(dimension[0], dimension[1], dimension[2]);
    //int size = pow(resolution, 3.0);
    //int square = pow(resolution, 2.0);
    // Calculate mass center of last point cloud and current point cloud
    int previous_count  = 0;
    int current_count   = 0;
    simd::float3 previous_mass_centre   = simd_make_float3(0, 0, 0);
    simd::float3 current_mass_centre    = simd_make_float3(0, 0, 0);
    for (int k=0; k<height*width; k++)
    {
        int i = k / width;
        int j = k % width;
        // Previous points
        float depth = previous_points[k];
        if ( ! std::isnan(depth) && depth >= 1e-6 ) {
            simd::float4 uv = simd_make_float4(depth * i, depth * j, depth, 1);
            simd::float4 local  = simd_mul(Kinv, uv);
            simd::float3 rlocal  = simd_make_float3(local.x, local.y, local.z);
            simd::float3 previous_point = simd_mul(simd_transpose(R), rlocal - T);
            previous_count ++;
            previous_mass_centre += previous_point;
        }
        // Current points
        depth = current_points[k];
        if ( ! std::isnan(depth) && depth >= 1e-6 ) {
            simd::float4 uv = simd_make_float4(depth * i, depth * j, depth, 1);
            simd::float4 local  = simd_mul(Kinv, uv);
            simd::float3 rlocal  = simd_make_float3(local.x, local.y, local.z);
            simd::float3 current_point = simd_mul(simd_transpose(R), rlocal - T);
            current_count ++;
            current_mass_centre += current_point;
        }
    }
    // Divide by number of points seen
    current_mass_centre     = current_mass_centre / (float) current_count;
    previous_mass_centre    = previous_mass_centre / (float) previous_count;
    // Add translation vector between current and previous points to previous transalation
    T  += (previous_mass_centre - current_mass_centre);
    //((simd_float4x4 *) translation)[0] = T;
    /*
     simd::float3 offset = 0.5 * dimensions;
     simd_float3x3 A = simd_diagonal_matrix(simd_make_float3(0,0,0));
     simd::float3 b  = simd_make_float3(0, 0, 0);
     for (int i = 0; i < height; i++) {
     for (int j = 0; j < width; j ++) {
     float depth = previous_points[ i*width + j];
     if (std::isnan(depth) || depth < 1e-6) continue;
     simd::float4 uv = simd_make_float4(depth * i, depth * j, depth, 1);
     simd::float4 point  = simd_mul(simd_mul(Rtinv, Kinv), uv);
     simd::float3 rpoint = simd_make_float3(point.x, point.y, point.z);
     simd_int3 V_ijk = global_to_integer(rpoint + offset, resolution, dimensions);
     int k = hash_function(V_ijk, resolution);
     int dkx = k + 1;
     int dky = k + resolution;
     int dkz = k + square;
     if (k >= size || k < 0) continue;
     if (dkx >= size || dkx < 0) continue;
     if (dky >= size || dky < 0) continue;
     if (dkz >= size || dkz < 0) continue;
     simd::float3 gradient = simd_make_float3(((Voxel *) voxels)[dkx].sdf - ((Voxel *) voxels)[k].sdf,
     ((Voxel *) voxels)[dky].sdf - ((Voxel *) voxels)[k].sdf,
     ((Voxel *) voxels)[dkz].sdf - ((Voxel *) voxels)[k].sdf);
     
     for (int n = 0; n < 3; n++)
     for (int m = 0; m < 3; m++)
     A.columns[n][m] += gradient[n] * gradient[m];
     b += ((Voxel *) voxels)[k].sdf * gradient;
     }
     }
     translation -= simd_mul(simd_inverse(A), b);
     ((simd_float4x4 *) extrinsics)[0].columns[3] = simd_make_float4(translation.x, translation.y, translation.z, 1);
     */
}
#endif /* icp_hpp */
