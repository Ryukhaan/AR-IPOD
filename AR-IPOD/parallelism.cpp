//
//  volumeInit.cpp
//  AR-IPOD
//
//  Created by Remi Decelle on 27/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

extern "C" {
    #include "parallelism.hpp"
}

#include <simd/common.h>
#include <simd/vector_types.h>
#include <simd/vector_make.h>
#include <simd/vector.h>
#include <vector>
//#include <omp.h>

using namespace std;
/*
 * LOCAL FUNCTIONS
 */
inline float integerToCenter(float point, int dim, float resolution) {
    return (resolution / (float) dim) * (0.5 + point);
}

simd::float3 interpolateVertex(float isolevel, simd::float3 a, simd::float3 b, float alpha, float beta) {
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
    
    if (values[0] < isolevel) cubeindex |= 1;
    if (values[1] < isolevel) cubeindex |= 2;
    if (values[2] < isolevel) cubeindex |= 4;
    if (values[3] < isolevel) cubeindex |= 8;
    if (values[4] < isolevel) cubeindex |= 16;
    if (values[5] < isolevel) cubeindex |= 32;
    if (values[6] < isolevel) cubeindex |= 64;
    if (values[7] < isolevel) cubeindex |= 128;
    /* Cube is entirely in/out of the surface */
    if (edgeTable[cubeindex] == 0) return triangles;
    
    /* Find the vertices where the surface intersects the cube */
    if (edgeTable[cubeindex] & 1) {
        vertexlist[0] = interpolateVertex(isolevel , points[0], points[1], values[0], values[1]);
    }
    if (edgeTable[cubeindex] & 2) {
        vertexlist[1] = interpolateVertex(isolevel, points[1], points[2], values[1], values[2]);
    }
    if (edgeTable[cubeindex] & 4) {
        vertexlist[2] = interpolateVertex( isolevel, points[2], points[3], values[2], values[3]);
    }
    if (edgeTable[cubeindex] & 8) {
        vertexlist[3] = interpolateVertex(isolevel, points[3], points[0], values[3], values[0]);
    }
    if (edgeTable[cubeindex] & 16) {
        vertexlist[4] = interpolateVertex(isolevel, points[4], points[5], values[4], values[5]);
    }
    if (edgeTable[cubeindex] & 32) {
        vertexlist[5] = interpolateVertex(isolevel, points[5], points[6], values[5], values[6]);
    }
    if (edgeTable[cubeindex] & 64) {
        vertexlist[6] = interpolateVertex(isolevel, points[6], points[7], values[6], values[7]);
    }
    if (edgeTable[cubeindex] & 128) {
        vertexlist[7] = interpolateVertex(isolevel, points[7], points[4], values[7], values[4]);
    }
    if (edgeTable[cubeindex] & 256) {
        vertexlist[8] = interpolateVertex(isolevel, points[0], points[4], values[0], values[4]);
    }
    if (edgeTable[cubeindex] & 512) {
        vertexlist[9] = interpolateVertex(isolevel, points[1], points[5], values[1], values[5]);
    }
    if (edgeTable[cubeindex] & 1024) {
        vertexlist[10] = interpolateVertex(isolevel, points[2], points[6], values[2], values[6]);
    }
    if (edgeTable[cubeindex] & 2048) {
        vertexlist[11] = interpolateVertex(isolevel, points[3], points[7], values[3], values[7]);
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
    
/*
 * GLOBAL FUNCTION (hpp)
 */
void bridge_initializeCentroids(void* centroids, int size, float resolution) {
//    int this_thread = omp_get_thread_num();
//    int num_threads = omp_get_num_threads();
    int count = size * size * size;
    int square = size * size;
//    printf("Num threads : %d", num_threads);
//#pragma omp parallel for shared(centroids) num_threads(num_threads)
    for(int i = 0; i<count; i++) {
        float x = i / square;
        float remainder = i % square;
        float y = remainder / size;
        float z = (float)(((int) remainder) % size);
        ((simd::float3*) centroids)[i][0] = integerToCenter(x, size, resolution);
        ((simd::float3*) centroids)[i][1] = integerToCenter(y, size, resolution);
        ((simd::float3*) centroids)[i][2] = integerToCenter(z, size, resolution);
    }
}

unsigned long bridge_extractMesh(void* triangles,
                            const float* voxels,
                            const void* centroids,
                            int edgeTable[256],
                            int triTable[4096],
                            int n,
                            float isolevel) {
    unsigned long index = 0;
    int n2 = n * n;
//    int num_threads = omp_get_num_threads();
//#pragma omp parallel for shared(index) collapse(3) num_threads(num_threads)
    for (int i = 0; i<n-1; i++) {
        for (int j = 0; j<n-1; j++) {
            for (int k = 0; k<n-1; k++) {
                int i0 = i*n2 + j*n + (k+1);
                int i1 = (i+1)*n2 + j*n + (k+1);
                int i2 = (i+1)*n2 + j*n + k;
                int i3 = i*n2 + j*n + k;
                int i4 = i*n2 + (j+1)*n + (k+1);
                int i5 = (i+1)*n2 + (j+1)*n + (k+1);
                int i6 = (i+1)*n2 + (j+1)*n + k;
                int i7 = i*n2 + (j+1)*n + k;
                simd::float3 points[8] = {
                    ((simd::float3*) centroids)[i0],
                    ((simd::float3*) centroids)[i1],
                    ((simd::float3*) centroids)[i2],
                    ((simd::float3*) centroids)[i3],
                    ((simd::float3*) centroids)[i4],
                    ((simd::float3*) centroids)[i5],
                    ((simd::float3*) centroids)[i6],
                    ((simd::float3*) centroids)[i7]
                };
                float values[8] = {
                    voxels[i0],
                    voxels[i1],
                    voxels[i2],
                    voxels[i3],
                    voxels[i4],
                    voxels[i5],
                    voxels[i6],
                    voxels[i7]
                };
                std::vector<simd::float3> temp = polygonise(points, values, isolevel, edgeTable, triTable);
                if (temp.size() == 0) continue;
                for (simd::float3 triangle : temp) {
                    ((simd::float3*) triangles)[index][0] = triangle[0];
                    ((simd::float3*) triangles)[index][1] = triangle[1];
                    ((simd::float3*) triangles)[index][2] = triangle[2];
//#pragma omp atomic
                    index++;
                }
            }
        }
    }
    return index;
}
