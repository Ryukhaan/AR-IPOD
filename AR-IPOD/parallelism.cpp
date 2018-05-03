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
#include <simd/conversion.h>
#include <simd/matrix.h>
#include <vector>
//#include <omp.h>

using namespace std;

typedef struct Voxel {
    float sdf;
    unsigned char weight;
} Voxel;

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

inline void updateVoxel(Voxel* voxels, const float sdf, const int weight, const int index) {
    float oldSDF      = voxels[index].sdf;
    float oldWeight   = voxels[index].weight;
    float newWeight   = oldWeight + weight;
    float oldProduct  = oldWeight * oldSDF;
    float newProduct  = sdf * weight;
    float newSDF      = (newProduct + oldProduct ) / newWeight;
    
    voxels[index].sdf     = newSDF;
    voxels[index].weight  = simd_min(newWeight, 255);
}


/*
inline void carveVoxel(float* sdfs, unsigned char* weight, const int index) {
    updateVoxel(sdfs, weight, 0.0, 1, index);
}
*/

inline simd_int2 project(simd::float3 vector, simd_float3x3 K) {
    simd::float3 temp = simd_make_float3(vector.x / vector.z, vector.y / vector.z, 1);
    simd::float3 all  = simd_mul(K, temp);
    return simd_make_int2((int)all.x, (int)all.y);
}

inline simd_float3 unproject(simd_int2 pixel, float depth, simd_float3x3 K) {
    simd_float3 temp = simd_make_float3(pixel.x, pixel.y, 1);
    simd_float3 all = simd_mul(K, temp);
    return simd_make_float3(all.x * depth, all.y * depth, depth);
}

/*
 * GLOBAL FUNCTION (hpp)
 */
void bridge_initializeCentroids(void* centroids, int size, float resolution) {
    //int this_thread = omp_get_thread_num();
    //int num_threads = omp_get_num_threads();
    int count = size * size * size;
    int square = size * size;
//    printf("Num threads : %d", num_threads);
#pragma omp parallel for shared(centroids) num_threads(num_threads)
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

int bridge_integrateDepthMap(const float* depthmap,
                              const void* centroids,
                              const void* camera_pose,
                              const void* intrisics,
                              void* voxels,
                              const int width,
                              const int height) {
    int number_of_changes = 0;
    float delta = 0.2;
    int count = width * height;
    // Convertion error !
    simd_float3x3 K = ((simd_float3x3 *) intrisics)[0];
    simd_float4x4 camera = ((simd_float4x4 *) camera_pose)[0];
    float diag = 2.0 * sqrt(3.0) * 1.0; // Resolution
    for (int i = 0; i<count; i++) {
        simd::float3 centroid = ((simd::float3 *) centroids)[i];
        simd::float3 position_camera = simd_make_float3(camera.columns[3][0], camera.columns[3][1], camera.columns[3][2]);
        simd_float4x3 temp = simd_transpose(simd_matrix(camera.columns[0], camera.columns[1], camera.columns[2]));
        simd_float3x3 R = simd_matrix_from_rows(temp.columns[0], temp.columns[1], temp.columns[2]);
        simd::float3 local = simd_mul(simd_transpose(R), centroid - position_camera);
        float z = local.z;
        //if (z < 0) continue;
        simd_int2 uv = project(local, K);
        if (uv.x > height || uv.x < 0) continue;
        if (uv.y > width || uv.y < 0) continue;
        float depth = depthmap[uv.x * width + uv.y];
        if (depth == 0.0) continue;

        float truncation = 2.0; // truncation distance
        float distance = depth - z;
        if (fabs(distance) < truncation + diag)
            updateVoxel((Voxel *)voxels, distance, 1, i);
        else if (distance >= truncation + diag)
            updateVoxel((Voxel *)voxels, delta, 1, i);
        else
            updateVoxel((Voxel *)voxels, -delta, 1, i);
    }
    return number_of_changes;
}
