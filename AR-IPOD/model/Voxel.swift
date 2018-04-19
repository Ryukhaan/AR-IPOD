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
    static let UNDEFINED_SDF: CGFloat = 9999
    
    var sdf:    CGFloat
    var weight: CGFloat
    
    init() {
        sdf     = Voxel.UNDEFINED_SDF
        weight  = 0
    }
    
    mutating func reset() {
        sdf     = Voxel.UNDEFINED_SDF
        weight  = 0
    }
    
    mutating func update(sdfUpdate: CGFloat, weightUpdate: CGFloat) {
        let oldSDF      = sdf
        let oldWeight   = weight
        let oldProduct  = oldWeight * oldSDF
        let newWeight   = (oldWeight + weightUpdate)
        let newProduct  = sdfUpdate * weightUpdate
        let newSDF      = (newProduct + oldProduct ) / newWeight
        
        sdf     = newSDF
        weight  = newWeight
    }
    
    mutating func carve() {
        update(sdfUpdate: 0.0, weightUpdate: 1)
    }
}
