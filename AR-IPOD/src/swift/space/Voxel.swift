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
    var time:   UInt8
    
    init() {
        sdf     = Voxel.UNDEFINED_SDF
        weight  = 0
        time    = 0
    }
}
