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
#include "constants.h"

static const double ang_min_sinc = 1.0e-8;
static const double ang_min_mc = 2.5e-4;

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

simd_float2x3 compute_bounding_box(float* depthmap,
                                   const int width,
                                   const int height,
                                   const simd_float3x3 rotation,
                                   const simd_float3 translation,
                                   const simd_float4x4 Kinv,
                                   const float cx,
                                   const float cy) {
    simd_float2x3 box = simd_matrix(simd_make_float3(99999, 99999, 99999), simd_make_float3(-99999, -99999, -99999));
    simd_float3x3 R = simd_transpose(rotation);
    //simd_float4x4 Rtinv;
    //for (int i = 0; i<3; i++) Rtinv.columns[i] = simd_make_float4(R.columns[i].x, R.columns[i].y, R.columns[i].z, 0);
    //Rtinv.columns[3] = simd_make_float4(-translation.x, -translation.y, -translation.z, 1);
    //Rtinv = simd_inverse(Rtinv);
    
    for (int i=0; i<height; i++) {
        for (int j=0; j<width; j++) {
            float depth = depthmap[i*width+j];
            if (std::isnan(depth) || depth < 1e-6) continue;
            /*
            simd::float4 uv = simd_make_float4(depth * j * cx, depth * i * cy, depth, 1);
            simd::float4 local = simd_mul(Kinv, uv);
            simd::float3 rlocal = simd_make_float3(local.x, local.y, local.z);
            simd::float3 world_point = simd_mul(simd_transpose(rotation), rlocal - translation);
            //simd::float3 world_point = simd_mul(rotation, rlocal) + translation;
            */
            simd::float4 uvz = homogenezie(j, i, depth, cx, cy);
            simd::float4 local = project_screen_to_local(uvz, Kinv);
            simd::float3 global = local_to_global(local, rotation, translation);
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
/*
simd_float3x3 rotation_from_lie(simd::float3 omega) {
    //
    simd_float3x3 R = simd_diagonal_matrix(simd_make_float3(1,1,1));
    R.columns[0].y = expf(omega.z);
    R.columns[0].z = expf(-omega.y);
    R.columns[1].x = expf(-omega.z);
    R.columns[1].z = expf(omega.x);
    R.columns[2].x = expf(omega.y);
    R.columns[2].y = expf(-omega.x);
    return R;
    //
    Eigen::Matrix3d R;
    R(0,0) = 0;
    R(0,1) = -omega.z;
    R(0,2) = omega.y;
    
    R(1,0) = omega.z;
    R(1,1) = 0;
    R(1,2) = -omega.x;
    
    R(2,0) = -omega.y;
    R(2,1) = omega.x;
    R(2,2) = 0;
    R = R.exp();
    
    return simd_matrix_from_rows(simd_make_float3(R(0,0), R(0,1), R(0,2)),
                                 simd_make_float3(R(1,0), R(1,1), R(1,2)),
                                 simd_make_float3(R(2,0), R(2,1), R(2,2)));
}
*/


/*
inline Vector3f project_camera_to_plane(const Vector3f point,
                                        const Matrix_4x4 K)
{
    Vector4f tmp = Vector4f(point(0), point(1), point(2), 1);
    return Vector3f(tmp(0) / tmp(2), tmp(1) / tmp(2), tmp(2));
}

inline Vector3f project_plane_to_camera(const Vector2i point,
                                        const float depth,
                                        const Matrix_4x4 K)
{
    Matrix_4x4 Kinv = K.inverse();
    Vector4f tmp = Kinv * (depth * Vector3f(point(0), point(1), 1.0));
    return Vector3f(tmp(0), tmp(1), tmp(2));
}
*/

inline double f_sinc(double sinx, double x)
{
    return (fabs(x) < ang_min_sinc) ? 1.0 : (sinx / x);
}

inline double f_mcosc(double cosx, double x)
{
    return (fabs(x) < ang_min_mc) ? 0.5 : ((1.0 - cosx) / x / x);
}

inline double f_msinc(double sinx, double x)
{
    return (fabs(x) < ang_min_mc) ? (1. / 6.0) : ((1.0 - sinx / x) / x / x);
}

#endif /* linalg_hpp */
