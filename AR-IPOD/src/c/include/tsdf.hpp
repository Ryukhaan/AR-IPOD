//
//  TSDF.hpp
//  AR-IPOD
//
//  Created by Remi Decelle on 09/05/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#ifndef tsdf_hpp
#define tsdf_hpp

#include <stdio.h>
#include <simd/common.h>
#include <simd/vector_types.h>
#include <simd/vector_make.h>
#include <simd/vector.h>
#include <simd/conversion.h>
#include <simd/matrix.h>

#include "types.h"

typedef struct Voxel {
    float sdf;
    float weight;
} Voxel;

inline float constant_weighting() {
    return 1.0;
}

inline float weighting(const float distance, const float delta, const float epsilon) {
    float w = 1.0;
    if (distance >= epsilon && distance <= delta)
        return exp(-0.5*(distance - epsilon)*(distance - epsilon));
    if (distance > delta)
        return 0;
    return w;
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
    voxels[index].weight  = new_weight;
};

inline void carving_voxel(Voxel * voxels, const int i) {
    if (voxels[i].sdf < 1e-5 && voxels[i].weight > 0) {
        voxels[i].sdf = 9999;
        voxels[i].weight = 0;
    }
};

inline Voxel update_voxel_at(Voxel voxel,
                             const float sdf,
                             const float weight) {
    Voxel new_voxel     = Voxel();
    float new_weight    = voxel.weight + weight;
    new_voxel.weight    = new_weight;
    new_voxel.sdf       = (voxel.sdf * voxel.weight + sdf * weight) / new_weight;
    return new_voxel;
}

inline Voxel carving_voxel_at(Voxel voxel) {
    if ( voxel.sdf < 1e-5 && voxel.weight > 0 ) {
        return Voxel();
    }
    return voxel;
}
#endif /* tsdf_hpp */
