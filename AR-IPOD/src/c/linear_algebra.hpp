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

#include "filtering.cpp"
/**
 * Mapping between integer to centroid coordinate.
 */
inline float integer_to_global(float point, int dim, float resolution) {
    return (resolution / dim) * (point + 0.5);
}

inline int global_to_integer(float point, int dim, float resolution) {
    return static_cast<int>(dim * point / resolution - 0.5);
}

inline simd_float3 create_centroid(const int i,
                                   const int dimension,
                                   const float resolution,
                                   const int square,
                                   const float offset) {
    simd::float3 centroid;
    int x = i / square;
    int remainder = i % square;
    int y = remainder / dimension;
    int z = remainder % dimension;
    centroid.x = integer_to_global(x, dimension, resolution) - offset;
    centroid.y = integer_to_global(y, dimension, resolution) - offset;
    centroid.z = integer_to_global(z, dimension, resolution);// - offset;
    return centroid;
}
/**
 * Project. A 3D point is porjected into the image plane (2D)
 */
inline simd_int2 project(simd::float3 vector, simd_float3x3 K) {
    simd::float3 temp = simd_make_float3(vector.x / vector.z, vector.y / vector.z, 1);
    simd::float3 all  = simd_mul(K, temp);
    return simd_make_int2( static_cast<int>(all.x), static_cast<int>(all.y));
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

inline int hash_function(simd_int3 point, int base) {
    int a = point.x * (base * base);
    int b = point.y * base;
    int c = point.z;
    return a + b + c;
}

simd_float2x3 bounding_box_and_filter(float* depthmap,
                           const int width,
                           const int height,
                           const simd_float3x3 rot,
                           const simd::float3 t,
                           const simd_float3x3 Kinv,
                           const int window_size) {
    simd_float2x3 box = simd_matrix(simd_make_float3(99999, 99999, 99999), simd_make_float3(-99999, -99999, -99999));
    for (int i=0; i<height; i++) {
        for (int j=0; j<width; j++) {
            float depth = depthmap[i*width+j];
            simd::float3 uv = simd_make_float3(i, j, 1);
            simd::float3 world_point = simd_mul(simd_transpose(rot), simd_mul(Kinv, depth * uv) - t);
            box.columns[0].x = simd_min(box.columns[0].x, world_point.x);
            box.columns[0].y = simd_min(box.columns[0].y, world_point.y);
            box.columns[0].z = simd_min(box.columns[0].z, world_point.z);
            box.columns[1].x = simd_max(box.columns[1].x, world_point.x);
            box.columns[1].y = simd_max(box.columns[1].y, world_point.y);
            box.columns[1].z = simd_max(box.columns[1].z, world_point.z);
            local_median(depthmap, window_size, width, height, i, j);
        }
    }
    return box;
}
#endif /* linear_algebra_hpp */
