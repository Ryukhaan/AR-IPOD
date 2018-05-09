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

inline simd::float3 projectWithZ(simd::float3 vector, simd_float3x3 K) {
    simd::float3 temp = simd_make_float3(vector.x, vector.y , vector.z);
    simd::float3 all  = simd_mul(K, temp);
    return simd_make_float3(all.x / all.z , all.y / all.z, all.z);
}

/**
 * Unproject. A pixel correspond to a 3D point
 */
inline simd_float3 unproject(simd_int2 pixel, float depth, simd_float3x3 K) {
    simd_float3 temp = simd_make_float3(pixel.x, pixel.y, 1);
    simd_float3 all = simd_mul(K, temp);
    return simd_make_float3(all.x * depth, all.y * depth, depth);
}


inline simd_float3 linear_interpolation(simd::float3 x, simd::float3 y, double mu) {
        return (1-mu) * x  + mu * y;
}

/**
 * Computes bilinear interpolation.
 */
inline simd_float3 bilinear_interpolation(simd::float3 c00,
                                          simd::float3 c01,
                                          simd::float3 c10,
                                          simd::float3 c11,
                                          float tx,
                                          float ty) {
    return linear_interpolation(linear_interpolation(c00, c10, tx),
                                linear_interpolation(c01, c11, tx),
                                ty);
}

/**
 * Computes trilinear interpolation with linear and bilinear interpolation.
 */
simd_float3 trilinear_interpolation(simd::float3 position) {
    simd::float3 gp = simd_make_float3(floor(position.x), floor(position.y), floor(position.z));
    float tx = (position.x - gp.x);
    float ty = (position.y - gp.y);
    float tz = (position.z - gp.z);
    simd::float3 c000 = simd_make_float3(gp.x, gp.y, gp.z);
    simd::float3 c100 = simd_make_float3(gp.x+1, gp.y, gp.z);
    simd::float3 c010 = simd_make_float3(gp.x, gp.y+1, gp.z);
    simd::float3 c110 = simd_make_float3(gp.x+1, gp.y+1, gp.z);
    simd::float3 c001 = simd_make_float3(gp.x, gp.y, gp.z+1);
    simd::float3 c101 = simd_make_float3(gp.x+1, gp.y, gp.z+1);
    simd::float3 c011 = simd_make_float3(gp.x, gp.y+1, gp.z+1);
    simd::float3 c111 = simd_make_float3(gp.x+1, gp.y+1, gp.z+1);
    simd::float3 e = bilinear_interpolation(c000, c100, c010, c110, tx, ty);
    simd::float3 f = bilinear_interpolation(c001, c101, c011, c111, tx, ty);
    return linear_interpolation(e, f, tz);
}

inline int hash_function(simd::float3 point, int base) {
    int a = (int) point.x * (base * base);
    int b = (int) point.y * base;
    int c = (int) point.z;
    return a + b + c;
}

#endif /* linear_algebra_hpp */
