//
//  Integrator.metal
//  AR-IPOD
//
//  Created by Remi Decelle on 26/07/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#include <metal_stdlib>

using namespace metal;

kernel void integrate(const device float *inVector [[ buffer(0) ]],
                      device float *outVector [[ buffer(1) ]],
                      uint3 id [[ thread_position_in_grid ]]) {
    //outVector[id] = 1.0 / (1.0 + exp(-inVector[id]));
    int index = id.x * 256 * 256 + id.y * 256 + id.z;
    float offset = 128.0;
    uint3 centroid = uint3(id.x - offset, id.y - offset, id.z - offset);
    outVector[index] = centroid.x + centroid.y + centroid.z;
}

