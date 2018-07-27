//
//  Integrator.metal
//  AR-IPOD
//
//  Created by Remi Decelle on 26/07/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#include <metal_stdlib>

#import <simd/simd.h>

using namespace metal;

float3 integer_to_global(int3 p, float resolution) {
    return float3(p.x * resolution,
                  p.y * resolution,
                  p.z * resolution);
}

kernel void integrate(const device float *inVector [[ buffer(0) ]],
                      device float3 *outVector [[ buffer(1) ]],
                      uint3 id [[ thread_position_in_grid ]]) {
    //outVector[id] = 1.0 / (1.0 + exp(-inVector[id]));
    // Hardcoded variables
    /*
    float resolution = 0.01;
    int n = 256;
    int n2 = n * n;
    float offset = 0.5 * n * resolution;
    
    int i = id.x;
    int j = id.y;
    int k = id.z;
    int idx = i * n2 + j * n + k;
    
    int i0 = idx;
    int i1 = idx + n2;
    int i2 = idx + n2 + 1;
    int i3 = idx + 1;
    int i4 = idx + n;
    int i5 = idx + n + n2;
    int i6 = idx + n + n2 + 1;
    int i7 = idx + n + 1;
    
    int3 p0 = int3(i, j, k);
    int3 p1 = int3(i+1, j, k);
    int3 p2 = int3(i+1, j, k+1);
    int3 p3 = int3(i, j, k+1);
    int3 p4 = int3(i, j+1, k);
    int3 p5 = int3(i+1, j+1, k);
    int3 p6 = int3(i+1, j+1, k+1);
    int3 p7 = int3(i, j+1, k+1);
    
    float3 c0 = integer_to_global(p0, resolution) - offset;
    float3 c1 = integer_to_global(p1, resolution) - offset;
    float3 c2 = integer_to_global(p2, resolution) - offset;
    float3 c3 = integer_to_global(p3, resolution) - offset;
    float3 c4 = integer_to_global(p4, resolution) - offset;
    float3 c5 = integer_to_global(p5, resolution) - offset;
    float3 c6 = integer_to_global(p6, resolution) - offset;
    float3 c7 = integer_to_global(p7, resolution) - offset;
    
    float3 points[8] = {c0, c1, c2, c3, c4, c5, c6, c7};
    */
    /*
    float values[8] = {
        ( ((Voxel*) voxels)[i0].sdf ),
        ( ((Voxel*) voxels)[i1].sdf ),
        ( ((Voxel*) voxels)[i2].sdf ),
        ( ((Voxel*) voxels)[i3].sdf ),
        ( ((Voxel*) voxels)[i4].sdf ),
        ( ((Voxel*) voxels)[i5].sdf ),
        ( ((Voxel*) voxels)[i6].sdf ),
        ( ((Voxel*) voxels)[i7].sdf )
    };
    std::vector<float3> temp = polygonise(points, values, isolevel, edgeTable, triTable);
    */
    //array<float3, 1> temp;
    //if (temp.size() <= 0) continue;
    
    /*
    void* ptr_realloc = realloc(mesh, sizeof(simd::float3) * (*ntriangles + temp.size()));
    if (ptr_realloc != NULL)
        mesh = (simd::float3 *)ptr_realloc;
    */
    /*
    for (float3 triangle : temp) {
        (*outVector)[index] = triangle
        index++;
    }
    */
}

