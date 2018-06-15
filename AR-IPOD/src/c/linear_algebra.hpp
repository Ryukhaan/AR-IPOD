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
inline float integer_to_global(int point, float resolution) {
    //return (resolution / dim) * (point + 0.5);
    return point * resolution;
}

inline int global_to_integer(float point, float resolution) {
    //return static_cast<int>(dim * point / resolution - 0.5);
    //return static_cast<int>((dim * point / resolution) - 0.5);
    return static_cast<int>(floor(point / resolution));
}

inline simd_int3 global_to_integer(simd::float3 point, simd::float3 resolution) {
    return simd_make_int3(global_to_integer(point.x, resolution.x),
                          global_to_integer(point.y, resolution.y),
                          global_to_integer(point.z, resolution.z));
}

inline simd_float3 integer_to_global(simd_int3 point, simd::float3 resolution) {
    return simd_make_float3(integer_to_global(point.x, resolution.x),
                            integer_to_global(point.y, resolution.y),
                            integer_to_global(point.z, resolution.z));
}

inline simd_int3 global_to_integer(simd::float3 point, float resolution) {
    return simd_make_int3(global_to_integer(point.x, resolution),
                          global_to_integer(point.y, resolution),
                          global_to_integer(point.z, resolution));
}

inline simd_float3 integer_to_global(simd_int3 point, float resolution) {
    return simd_make_float3(integer_to_global(point.x, resolution),
                            integer_to_global(point.y, resolution),
                            integer_to_global(point.z, resolution));
}

inline int hash_code(simd_int3 point, int base) {
    int a = point.x * (base * base);
    int b = point.y * base;
    int c = point.z;
    return a + b + c;
}

inline simd_int3 hash_decode(int i, int base) {
    int square = base * base;
    int x = i / square;
    int remainder = i - square * x;
    int y = remainder / base;
    int z = remainder - y * base;
    return simd_make_int3(x, y, z);
}

inline simd_float3 create_centroid(const int i,
                                   const float resolution,
                                   const int dimension) {
    float offset = resolution * 0.5;
    simd_int3 coord = hash_decode(i, dimension);
    return integer_to_global(coord, resolution) + offset;
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

simd_float2x3 compute_bounding_box(float* depthmap,
                                   const int width,
                                   const int height,
                                   const simd_float3x3 rotation,
                                   const simd_float3 translation,
                                   const simd_float4x4 Kinv) {
    simd_float2x3 box = simd_matrix(simd_make_float3(99999, 99999, 99999), simd_make_float3(-99999, -99999, -99999));
    for (int i=0; i<height; i++) {
        for (int j=0; j<width; j++) {
            float depth = depthmap[i*width+j];
            if (std::isnan(depth) || depth < 1e-6) continue;
            simd::float4 uv = simd_make_float4(depth * i, depth * j, depth, 1);
            simd::float4 local = simd_mul(Kinv, uv);
            simd::float3 rlocal = simd_make_float3(local.x, local.y, local.z);
            //simd::float3 world_point = simd_mul(simd_transpose(rotation), rlocal - translation);
            simd::float3 world_point = simd_mul(rotation, rlocal + translation);
            box.columns[0].x = simd_min(box.columns[0].x, world_point.x);
            box.columns[0].y = simd_min(box.columns[0].y, world_point.y);
            box.columns[0].z = simd_min(box.columns[0].z, world_point.z);
            box.columns[1].x = simd_max(box.columns[1].x, world_point.x);
            box.columns[1].y = simd_max(box.columns[1].y, world_point.y);
            box.columns[1].z = simd_max(box.columns[1].z, world_point.z);
        }
    }
    return box;
}
#endif /* linear_algebra_hpp */
