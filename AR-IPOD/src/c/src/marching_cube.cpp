//
//  marching_cube.cpp
//  AR-IPOD
//
//  Created by Remi Decelle on 04/05/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//


#include "marching_cube.hpp"
#include <simd/common.h>
#include <simd/vector_types.h>
#include <simd/vector_make.h>
#include <simd/vector.h>
#include <simd/conversion.h>
#include <simd/matrix.h>
#include <vector>

simd::float3 interpolate(float isolevel, simd::float3 a, simd::float3 b, float alpha, float beta) {
    /* First Version --
    if (abs(isolevel-alpha) < 0.00001)  { return a; }
    if (abs(isolevel-beta) < 0.00001)   { return b; }
    if (abs(alpha-beta) < 0.00001)      { return a; }
    float mu = (isolevel - alpha) / (beta - alpha);
    return a + mu * (b - a);
    */
    // Chisel & Bylow Version --
    const float min_diff = 1e-6;
    const float sdf_diff = alpha - beta;
    const float mu = alpha / sdf_diff;
    return (fabs(sdf_diff) < min_diff) ? a + 0.5 * b : a + mu * (b - a);
}

int calculate_vertex_configuration(const float values[8], const float isolevel) {
    return (values[0] < isolevel ? (1<<0) : 0) |
    (values[1] < isolevel ? (1<<1) : 0) |
    (values[2] < isolevel ? (1<<2) : 0) |
    (values[3] < isolevel ? (1<<3) : 0) |
    (values[4] < isolevel ? (1<<4) : 0) |
    (values[5] < isolevel ? (1<<5) : 0) |
    (values[6] < isolevel ? (1<<6) : 0) |
    (values[7] < isolevel ? (1<<7) : 0);
}

std::vector<simd::float3> polygonise(const simd::float3 points[8],
                                     const float values[8],
                                     const float isolevel,
                                     const int edgeTable[256],
                                     const int triTable[4096]) {
    std::vector<simd::float3> triangles;
    //int ntriangle = 0;
    // int cubeindex = 0;
    int cubeindex = calculate_vertex_configuration(values, isolevel);
    simd::float3 vertexlist[12];
    simd_int2 pairs[12] = {
        simd_make_int2(0, 1),
        simd_make_int2(1, 2),
        simd_make_int2(2, 3),
        simd_make_int2(3, 0),
        simd_make_int2(4, 5),
        simd_make_int2(5, 6),
        simd_make_int2(6, 7),
        simd_make_int2(7, 4),
        simd_make_int2(0, 4),
        simd_make_int2(1, 5),
        simd_make_int2(2, 6),
        simd_make_int2(3, 7)
    };
    
    for (int i = 0; i < 12; i++) {
        const int i0 = pairs[i].x;
        const int i1 = pairs[i].y;
        if ( (values[i0] < isolevel && values[i1] >= isolevel) || (values[i0] >= isolevel && values[i1] < isolevel))
            vertexlist[i] = interpolate(isolevel , points[i0], points[i1], values[i0], values[i1]);
    }
    
    int i = 0;
    while (triTable[cubeindex*16+i] != -1) {
        const simd::float3 t1 = vertexlist[triTable[cubeindex*16+i+0]];
        const simd::float3 t2 = vertexlist[triTable[cubeindex*16+i+1]];
        const simd::float3 t3 = vertexlist[triTable[cubeindex*16+i+2]];
        triangles.push_back(t1);
        triangles.push_back(t2);
        triangles.push_back(t3);
        //ntriangle += 1;
        i += 3;
    }
    return triangles;
}
