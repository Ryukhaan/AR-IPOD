//
//  linear_algebra.hpp
//  AR-IPOD
//
//  Created by Remi Decelle on 04/05/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#ifndef linear_algebra_hpp
#define linear_algebra_hpp

#include <stdio.h>
#include <simd/common.h>
#include <simd/vector_types.h>
#include <simd/vector_make.h>
#include <simd/vector.h>
#include <simd/conversion.h>
#include <simd/matrix.h>


/**
 * Mapping between integer to centroid coordinate.
 */
inline float integer_to_global(float point, int dim, float resolution) {
    return (resolution / (float) dim) * (0.5 + point);
}

/**
 * Mapping between world and voxel coordinate
 * dimension = m (Bylow)
 * resolution = width x height x width
 */
inline simd_float3 mapping_global_to_voxel(simd::float3 x, int dimension, simd::float3 resolution) {
    return dimension * simd_make_float3(x.x / resolution.x + 0.5,
                                        x.y / resolution.y + 0.5,
                                        x.z / resolution.z + 0.5);
}


/**
 * Project. A 3D point is porjected into the image plane (2D)
 */
inline simd_int2 project(simd::float3 vector, simd_float3x3 K) {
    simd::float3 temp = simd_make_float3(vector.x / vector.z, vector.y / vector.z, 1);
    simd::float3 all  = simd_mul(K, temp);
    return simd_make_int2((int)all.x, (int)all.y);
}

/**
 * Unproject. A pixel correspond to a 3D point
 */
inline simd_float3 unproject(simd_int2 pixel, float depth, simd_float3x3 K) {
    simd_float3 temp = simd_make_float3(pixel.x, pixel.y, 1);
    simd_float3 all = simd_mul(K, temp);
    return simd_make_float3(all.x * depth, all.y * depth, depth);
}

#endif /* linear_algebra_hpp */
