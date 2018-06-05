//
//  TSDF.cpp
//  AR-IPOD
//
//  Created by Remi Decelle on 09/05/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#include "TSDF.hpp"
#include <simd/common.h>
#include <simd/vector_types.h>
#include <simd/vector_make.h>
#include <simd/vector.h>
#include <simd/conversion.h>
#include <simd/matrix.h>
#include <vector>

/*
 * LOCAL FUNCTIONS
 */

/**
 * Update voxel sdf and weight.
 */
inline void update_voxel(Voxel* voxels, const float sdf, const int weight, const int index) {
    float old_sdf      = voxels[index].sdf;
    float old_weight   = voxels[index].weight;
    float new_weight   = old_weight + weight;
    float old_product  = old_weight * old_sdf;
    float new_product  = sdf * weight;
    float new_sdf      = (new_product + old_product ) / new_weight;
    
    voxels[index].sdf     = new_sdf;
    voxels[index].weight  = simd_min(new_weight, 100);
}

/**
 * Reset voxel
 */
inline void reset_voxel(Voxel * voxels, const int i) {
    voxels[i].sdf = 9999;
    voxels[i].weight = 0;
}