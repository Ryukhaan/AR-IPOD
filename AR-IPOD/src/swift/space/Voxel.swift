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
    static let UNDEFINED_SDF: Float = 9999.0
    
    var sdf:    Float
    var weight: UInt8
    
    init() {
        sdf     = Voxel.UNDEFINED_SDF
        weight  = 0
    }
    
    mutating func reset() {
        sdf     = Voxel.UNDEFINED_SDF
        weight  = 0
    }
    
    mutating func update(sdfUpdate: Float, weightUpdate: UInt8) {
        let oldSDF      = sdf
        let oldWeight   = weight
        let newWeight   = oldWeight + weightUpdate
        let oldProduct  = Float(oldWeight) * oldSDF
        let newProduct  = sdfUpdate * Float(weightUpdate)
        let newSDF      = (newProduct + oldProduct ) / Float(newWeight)
        
        sdf     = newSDF
        weight  = newWeight < 100 ? newWeight : 100
    }
    
    mutating func carve() {
        update(sdfUpdate: 0.0, weightUpdate: 1)
    }
}
