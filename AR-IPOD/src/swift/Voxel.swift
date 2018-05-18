//
//  Voxels.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 19/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit
import AVFoundation

struct Voxel {
    var sdf:    Float
    var weight: Float
    
    init() {
        sdf     = 9999.0
        weight  = 0
    }
    
    mutating func update(sdf: Float, weight: Float) {
        let old_sdf      = self.sdf;
        let old_weight   = self.weight;
        
        let new_weight   = old_weight + weight;
        let old_product  = old_weight * old_sdf;
        let new_product  = sdf * weight;
        
        let new_sdf      = (new_product + old_product ) / new_weight;
        self.sdf = new_sdf;
        self.weight = new_weight;
    }
}
