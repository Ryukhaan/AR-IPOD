//
//  linalg.hpp
//  AR-IPOD
//
//  Created by Remi Decelle on 04/05/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#ifndef linalg_hpp
#define linalg_hpp

#include <stdio.h>
#include <simd/common.h>
#include <simd/vector_types.h>
#include <simd/vector_make.h>
#include <simd/vector.h>
#include <simd/conversion.h>
#include <simd/matrix.h>

//#include <Eigen/Dense>
//#include <unsupported/Eigen/MatrixFunctions>

#include "filtering.hpp"
#include "omp.h"

/**
 * Mapping between integer to centroid coordinate.
 */
inline float integer_to_global(int point, float resolution) {
    return point * resolution;
}

inline int global_to_integer(float point, float resolution) {
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
    int a = point.x * (base * base); // point.x
    int b = point.y * base; // point.y
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
    //float offset = resolution * 0.5;
    simd_int3 coord = hash_decode(i, dimension);
    return integer_to_global(coord, resolution); //+ offset;
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

inline simd::float4 homogenezie(const int i,
                                const int j,
                                const float depth,
                                const float cx,
                                const float cy) {
    return simd_make_float4(depth * j * cx, depth * i * cy, depth, 1);
}

inline simd::float4 project_screen_to_local(simd::float4 homogeneous, simd_float4x4 Kinv) {
    return simd_mul(Kinv, homogeneous);
}

inline simd::float4 unproject_local_to_screen(simd::float4 local, simd_float4x4 K) {
    return simd_mul(K, local);
}

inline simd::float4 local_to_global(simd::float4 local, simd_float4x4 Rtinv) {
    return simd_mul(Rtinv, local);
}

inline simd::float3 local_to_global(simd::float4 local, simd_float3x3 R, simd::float3 T) {
    simd::float3 tmp = simd_make_float3(local.x, local.y, local.z);
    return simd_mul(simd_transpose(R), tmp - T);
}

inline simd::float4 global_to_local(simd::float4 global, simd_float4x4 Rt) {
    return simd_mul(Rt, global);
}

inline simd::float3 global_to_local(simd::float3 global, simd_float3x3 R, simd::float3 T) {
    return simd_mul(R, global) + T;
}

simd_float2x3 compute_bounding_box(const float* depthmap,
                                   const int width,
                                   const int height,
                                   const simd_float4x4 Rt,
                                   //const simd_float3 translation,
                                   const simd_float4x4 Kinv,
                                   const float cx,
                                   const float cy) {
    float FINF = 65535.0;
    simd_float2x3 box = simd_matrix(simd_make_float3(FINF, FINF, FINF),
                                    simd_make_float3(-FINF, -FINF, -FINF));
    simd_float4x4 passage_global = simd_matrix(
                                               simd_make_float4( 0, -1,  0,  0),
                                               simd_make_float4( 1,  0,  0,  0),
                                               simd_make_float4( 0,  0,  1,  0),
                                               simd_make_float4( 0,  0,  0,  1));
    simd_float4x4 passage_local = simd_matrix(
                                              simd_make_float4( 1,  0,  0,  0),
                                              simd_make_float4( 0, -1,  0,  0),
                                              simd_make_float4( 0,  0, -1,  0),
                                              simd_make_float4( 0,  0,  0,  1));
    int num_threads = omp_get_max_threads();
    omp_set_num_threads(num_threads);
#pragma omp parallel for collapse(2)
    for (int i=0; i<height; i++) {
        for (int j=0; j<width; j++) {
            float depth = depthmap[i*width+j];
            if (std::isnan(depth) || depth < 1e-6) continue;
            
            simd::float4 uv = simd_make_float4(depth * j * cx, depth * i * cy, depth, 1);
            simd::float4 local = simd_mul(Kinv, uv);
            local = simd_mul(passage_local, local);
            simd::float4 global = simd_mul(Rt, local);
            global = simd_mul(passage_global, global);
            //simd::float3 rlocal = simd_make_float3(local.x, local.y, local.z);
            //simd::float3 global = simd_mul(rotation, rlocal) + translation;
            
            
            box.columns[0].x = simd_min(box.columns[0].x, global.x);
            box.columns[0].y = simd_min(box.columns[0].y, global.y);
            box.columns[0].z = simd_min(box.columns[0].z, global.z);
            box.columns[1].x = simd_max(box.columns[1].x, global.x);
            box.columns[1].y = simd_max(box.columns[1].y, global.y);
            box.columns[1].z = simd_max(box.columns[1].z, global.z);
        }
    }
    return box;
}

#endif /* linalg_hpp */
