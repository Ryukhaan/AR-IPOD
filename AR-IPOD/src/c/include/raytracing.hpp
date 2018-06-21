//
//  raytracing.hpp
//  AR-IPOD
//
//  Created by Remi Decelle on 11/06/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#ifndef raytracing_hpp
#define raytracing_hpp

#include <stdio.h>
#include <simd/common.h>
#include <simd/vector_types.h>
#include <simd/vector_make.h>
#include <simd/vector.h>
#include <simd/conversion.h>
#include <simd/matrix.h>

void raytracing(float* depthmap,
                simd_float3x3 R,
                simd_float3 T,
                simd_float4x4 Kinv,
                Voxel* voxels,
                const int width,
                const int height,
                const int dimension,
                const float resolution,
                const float delta,
                const float epsilon,
                const float lambda) {
    int size = pow(dimension, 3.0);
    simd::float3 offset = 0.5 * (dimension * resolution);
    simd_int3 ot        = global_to_integer(T + offset, resolution);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            float depth = depthmap[ i*width + j];
            if (std::isnan(depth) || depth < 1e-6) continue;
            simd::float4 uv = simd_make_float4(depth * i, depth * j, depth, 1);
            simd::float4 local      = simd_mul(Kinv,uv);
            simd::float3 rlocal     = simd_make_float3(local.x, local.y, local.z);
            simd::float3 rglobal    = simd_mul(R, rlocal + T);
            simd_int3 end           = global_to_integer(rglobal + offset, resolution);
            simd_int3 current       = ot;
            //float maxDist = simd_length(direction);
            float dx = rglobal.x - T.x;
            float dy = rglobal.y - T.y;
            float dz = rglobal.z - T.z;
            
            /*
            float t1 = - T.x / dx;
            float t2 = - T.y / dy;
            float t3 = - T.z / dz;
            float t4 = (dimension - T.x) / dx;
            float t5 = (dimension - T.y) / dy;
            float t6 = (dimension - T.z) / dz;
            */
            
            int stepX = dx == 0 ? 0 : dx > 0 ? 1 : -1;
            int stepY = dy == 0 ? 0 : dy > 0 ? 1 : -1;
            int stepZ = dz == 0 ? 0 : dz > 0 ? 1 : -1;
            
            if (stepX == 0 && stepY == 0 && stepZ == 0) return;
            while (true) {
                if (current.x == end.x && current.y == end.y && current.z == end.z) break;
                int k = hash_code(current, dimension);
                if (k < size && k >= 0) {
                    simd::float3 c = (integer_to_global(current, resolution) + resolution * 0.5) - offset;
                    //float distance = rglobal.z - c.z; //c.z - global.z;
                    float distance = simd_sign(rglobal.z - c.z) * simd_distance(rglobal, c);
                    float w = constant_weighting();
                    //if (distance >= delta + epsilon)  carving_voxel(voxels, i);
                    //if (fabs(distance) <= delta) update_voxel(voxels, distance, w, k);
                    //else if (distance > delta) update_voxel(voxels, delta, 1.0, k);
                    //else if (distance < -delta) update_voxel(voxels, -delta, 1.0, k);
                    //if (fabs(distance) <= delta) update_voxel((Voxel *)voxels, distance, w, i);
                }
                if (current.x != end.x)
                    current.x += stepX;
                if (current.y != end.y)
                    current.y += stepY;
                if (current.z != end.z)
                    current.z += stepZ;
            }
        }
    }
};
#endif /* raytracing_hpp */
