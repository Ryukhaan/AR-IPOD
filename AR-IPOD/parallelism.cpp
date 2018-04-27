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

float integerToCenter(float point, int dim, float resolution) {
    return (resolution / (float) dim) * (0.5 + point);
}

void bridge_initializeCentroids(void* centroids, int size, float resolution) {
    int count = size * size * size;
    int square = size * size;
//#pragma omp parallel num_threads(4) for
    for(int i = 0; i<count; i++) {
        float x = i / square;
        float remainder = i % square;
        float y = remainder / size;
        float z = (float)(((int) remainder) % size);
        ((vector_float3*) centroids)[i][0] = integerToCenter(x, size, resolution);
        ((vector_float3*) centroids)[i][1] = integerToCenter(y, size, resolution);
        ((vector_float3*) centroids)[i][2] = integerToCenter(z, size, resolution);
    }
}
