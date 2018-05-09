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
    if (abs(isolevel-alpha) < 0.00001)  { return a; }
    if (abs(isolevel-beta) < 0.00001)   { return b; }
    if (abs(alpha-beta) < 0.00001)      { return a; }
    float mu = (isolevel - alpha) / (beta - alpha);
    return a + mu * (b - a);
}

std::vector<simd::float3> polygonise(simd::float3 points[8], float values[8], float isolevel, int* edgeTable, int* triTable) {
    std::vector<simd::float3> triangles;
    int ntriangle = 0;
    int cubeindex = 0;
    simd::float3 vertexlist[12];
    
    if (values[0] <= isolevel) cubeindex |= 1;
    if (values[1] <= isolevel) cubeindex |= 2;
    if (values[2] <= isolevel) cubeindex |= 4;
    if (values[3] <= isolevel) cubeindex |= 8;
    if (values[4] <= isolevel) cubeindex |= 16;
    if (values[5] <= isolevel) cubeindex |= 32;
    if (values[6] <= isolevel) cubeindex |= 64;
    if (values[7] <= isolevel) cubeindex |= 128;
    /* Cube is entirely in/out of the surface */
    if (edgeTable[cubeindex] == 0) return triangles;
    
    /* Find the vertices where the surface intersects the cube */
    if (edgeTable[cubeindex] & 1) {
        vertexlist[0] = interpolate(isolevel , points[0], points[1], values[0], values[1]);
    }
    if (edgeTable[cubeindex] & 2) {
        vertexlist[1] = interpolate(isolevel, points[1], points[2], values[1], values[2]);
    }
    if (edgeTable[cubeindex] & 4) {
        vertexlist[2] = interpolate( isolevel, points[2], points[3], values[2], values[3]);
    }
    if (edgeTable[cubeindex] & 8) {
        vertexlist[3] = interpolate(isolevel, points[3], points[0], values[3], values[0]);
    }
    if (edgeTable[cubeindex] & 16) {
        vertexlist[4] = interpolate(isolevel, points[4], points[5], values[4], values[5]);
    }
    if (edgeTable[cubeindex] & 32) {
        vertexlist[5] = interpolate(isolevel, points[5], points[6], values[5], values[6]);
    }
    if (edgeTable[cubeindex] & 64) {
        vertexlist[6] = interpolate(isolevel, points[6], points[7], values[6], values[7]);
    }
    if (edgeTable[cubeindex] & 128) {
        vertexlist[7] = interpolate(isolevel, points[7], points[4], values[7], values[4]);
    }
    if (edgeTable[cubeindex] & 256) {
        vertexlist[8] = interpolate(isolevel, points[0], points[4], values[0], values[4]);
    }
    if (edgeTable[cubeindex] & 512) {
        vertexlist[9] = interpolate(isolevel, points[1], points[5], values[1], values[5]);
    }
    if (edgeTable[cubeindex] & 1024) {
        vertexlist[10] = interpolate(isolevel, points[2], points[6], values[2], values[6]);
    }
    if (edgeTable[cubeindex] & 2048) {
        vertexlist[11] = interpolate(isolevel, points[3], points[7], values[3], values[7]);
    }
    
    int i = 0;
    while (triTable[cubeindex*16+i] != -1) {
        simd::float3 triangle;
        triangle = vertexlist[triTable[cubeindex*16+i+0]];
        triangles.push_back(triangle);
        triangle = vertexlist[triTable[cubeindex*16+i+1]];
        triangles.push_back(triangle);
        triangle = vertexlist[triTable[cubeindex*16+i+2]];
        triangles.push_back(triangle);
        ntriangle += 1;
        i += 3;
    }
    return triangles;
}
