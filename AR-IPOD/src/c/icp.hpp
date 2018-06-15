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


simd::float3x3 g(simd::float3 omega, simd::float3 t, simd::float3 x) {
    simd_float3x3 omega_tilde = simd_diagonal_matrix(simd_make_float3(0,0,0));
    omega_tilde.columns[0].y = omega.z;
    omega_tilde.columns[0].z = -omega.y;
    omega_tilde.columns[1].x = -omega.z;
    omega_tilde.columns[1].z = omega.x;
    omega_tilde.columns[2].x = omega.y;
    omega_tilde.columns[2].y = -omega.x;
    return omega_tilde;
}
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
    //simd::float3  R = ((simd_float3 *) rotation)[0];
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
            //simd::float3 previous_point = simd_mul(simd_transpose(R), rlocal - T);
            simd::float3 previous_point = simd_mul(R, rlocal + T);
            previous_count ++;
            previous_mass_centre += previous_point;
        }
        // Current points
        depth = current_points[k];
        if ( ! std::isnan(depth) && depth >= 1e-6 ) {
            simd::float4 uv = simd_make_float4(depth * i, depth * j, depth, 1);
            simd::float4 local  = simd_mul(Kinv, uv);
            simd::float3 rlocal  = simd_make_float3(local.x, local.y, local.z);
            //simd::float3 current_point = simd_mul(simd_transpose(R), rlocal - T);
            simd::float3 current_point = simd_mul(R, rlocal + T);
            current_count ++;
            current_mass_centre += current_point;
        }
    }
    // Divide by number of points seen
    current_mass_centre     = current_mass_centre / (float) current_count;
    previous_mass_centre    = previous_mass_centre / (float) previous_count;
    // Add translation vector between current and previous points to previous transalation
    T  += (previous_mass_centre - current_mass_centre);
    ((simd_float3 *) translation)[0] = T;
    /*
    simd::float3 offset = 0.5 * dimension * resolution;
    int square = dimension * dimension;
    int size = pow(dimension, 3);
    
    simd_float3x3 A = simd_diagonal_matrix(simd_make_float3(0,0,0));
    simd::float3 b  = simd_make_float3(0, 0, 0);
    
    simd::float3 T_old = T;
    simd::float3 T_new = T;
    simd::float3 W_old = R;
    simd::float3 W_new = R;
    while (true) {
        simd_float3x3 A = simd_diagonal_matrix(simd_make_float3(0,0,0));
        simd::float3 b  = simd_make_float3(0, 0, 0);
        T_old = T_new;
        W_old = W_new;
        
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j ++) {
                float depth = previous_points[ i*width + j];
                if (std::isnan(depth) || depth < 1e-6) continue;
                simd::float4 uv = simd_make_float4(depth * i, depth * j, depth, 1);
                simd::float4 local  = simd_mul(Kinv, uv);
                simd::float3 global = simd_mul(R_new, simd_make_float3(local.x, local.y, local.z) + T_new);
                simd_int3 V_ijk = global_to_integer(global + offset, resolution);
                int k = hash_code(V_ijk, resolution);
                int dkx = k + square;
                int dky = k + dimension;
                int dkz = k + 1;
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
        T_new -= simd_mul(simd_inverse(A), b);
        W_new -= simd_mul(simd_inverse(A), b);
        if (std::isnan(T_new.x) || std::isnan(T_new.y) || std::isnan(T_new.z))
            break;
        if (simd_max(simd_norm_inf(T_new - T_old), simd_norm_inf(W_new - W_old)) < 1e-2)
            break;
    }
    R_new = simd_diagonal_matrix(simd_make_float3(1, 1, 1));
    R_new.columns[0].y = expf(W_old.z);   R_new.columns[1].x = expf(-W_old.z);  R_new.columns[2].x = expf(W_old.y);
    R_new.columns[0].z = expf(-W_old.y);  R_new.columns[1].z = expf( W_old.x);  R_new.columns[0].y = expf(-W_old.x);
                                                            
    ((simd_float3x3 *) rotation)[0] = R_new;
    ((simd_float3 *) translation)[0] = T_new;
    */
}
#endif /* icp_hpp */
