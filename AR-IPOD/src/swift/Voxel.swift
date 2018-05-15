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
}
