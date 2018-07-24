//
//  TSDF.hpp
//  AR-IPOD
//
//  Created by Remi Decelle on 09/05/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#ifndef tsdf_hpp
#define tsdf_hpp

#include <stdio.h>
#include <simd/common.h>
#include <simd/vector_types.h>
#include <simd/vector_make.h>
#include <simd/vector.h>
#include <simd/conversion.h>
#include <simd/matrix.h>

typedef struct Voxel {
    float sdf;
    float weight;
} Voxel;

inline float constant_weighting() {
    return 1.0;
}

inline float weighting(const float distance, const float delta, const float epsilon) {
    float w = 1.0;
    if (distance >= epsilon && distance <= delta)
        return exp(-0.5*(distance - epsilon)*(distance - epsilon));
    if (distance > delta)
        return 0;
    return w;
}

inline Voxel update_voxel(Voxel voxel,
                          const float sdf,
                          const float weight) {
    Voxel new_voxel     = Voxel();
    float new_weight    = voxel.weight + weight;
    new_voxel.weight    = new_weight;
    new_voxel.sdf       = (voxel.sdf * voxel.weight + sdf * weight) / new_weight;
    return new_voxel;
}

inline Voxel carving_voxel(Voxel voxel) {
    if ( voxel.sdf < 1e-5 && voxel.weight > 0 ) {
        return Voxel();
    }
    return voxel;
}

float interpolate_distance(const Voxel* voxels,
                           const simd::int3 voxel_coordinates,
                           const int dimension) {
    int squared = pow(dimension, 2.0);
    int size    = pow(dimension, 3.0);
    float i = voxel_coordinates.x;
    float j = voxel_coordinates.y;
    float k = voxel_coordinates.z;
    float w_sum = 0.0;
    float sum_d = 0.0;
    simd_int3 current_voxel;
    float w = 0;
    float volume;
    int a_idx;
    for (int i_offset = 0; i_offset < 2; i_offset++)
    {
        for (int j_offset = 0; j_offset < 2; j_offset++)
        {
            for (int k_offset = 0; k_offset < 2; k_offset++)
            {
                current_voxel.x = ((int) i)+i_offset;
                current_voxel.y = ((int) j)+j_offset;
                current_voxel.z = ((int) k)+k_offset;
                //volume = fabs(current_voxel.x-i) + fabs(current_voxel.y-j)+ fabs(current_voxel.z-k);
                volume  = i_offset + j_offset + k_offset;
                a_idx   = current_voxel.x * squared + current_voxel.y * dimension + current_voxel.z;
                if (a_idx < 0 || a_idx >= size) continue;
                if (voxels[a_idx].weight <= 0) continue;
                if (volume < 1e-5) return voxels[a_idx].sdf;
                w = 1.0 / volume;
                w_sum += w;
                sum_d += w * voxels[a_idx].sdf;
            }
        }
    }
    return (w_sum == 0.0) ? 0 : sum_d / w_sum;
}
#endif /* tsdf_hpp */
