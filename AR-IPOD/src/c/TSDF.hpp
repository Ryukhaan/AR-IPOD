//
//  TSDF.hpp
//  AR-IPOD
//
//  Created by Remi Decelle on 09/05/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#ifndef TSDF_hpp
#define TSDF_hpp

#include <stdio.h>
#include <simd/common.h>
#include <simd/vector_types.h>
#include <simd/vector_make.h>
#include <simd/vector.h>
#include <simd/conversion.h>
#include <simd/matrix.h>

typedef struct Voxel {
    float sdf;
    //unsigned char weight;
    float weight;
    //unsigned char time;
} Voxel;

inline float constant_weighting(const float distance, const float delta, const float lambda) {
    if ( distance < lambda ) return 1.0;
    else if (distance > delta) return 0.0;
    else return (delta - distance) / (delta - lambda);
}
inline void update_voxel(Voxel* voxels,
                         const float sdf,
                         const float weight,
                         const int index) {
    float old_sdf      = voxels[index].sdf;
    float old_weight   = voxels[index].weight;
    float new_weight   = old_weight + weight;
    float old_product  = old_weight * old_sdf;
    float new_product  = sdf * weight;
    float new_sdf      = (new_product + old_product ) / new_weight;
    
    // DWRAFing
    /*
    float delta = new_sdf - old_sdf;
    if (delta > 0 && delta < 0.5)
        voxels[index].sdf     = old_sdf+1;
    else if (delta < 0 && delta > -0.5)
        voxels[index].sdf     = old_sdf-1;
    else
    */
    voxels[index].sdf     = new_sdf;
    voxels[index].weight  = simd_min(new_weight, 100);
    //voxels[index].time    = 0;
};

inline void carving_voxel(Voxel * voxels, const int i) {
    if (voxels[i].sdf <= 0.0) {
        voxels[i].sdf = 9999;
        voxels[i].weight = 0;
    }
};

/*
inline void olding_voxel(Voxel * voxels, const int i, const unsigned char time_constant) {
    if (voxels[i].time >= time_constant) carving_voxel(voxels, i);
    else voxels[i].time ++;
}
*/
#endif /* TSDF_hpp */
