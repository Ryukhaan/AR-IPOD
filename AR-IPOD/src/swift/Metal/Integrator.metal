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
                      uint id [[ thread_position_in_grid ]]) {
}

