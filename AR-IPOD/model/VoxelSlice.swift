//
//  VoxelSlice.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 19/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit
import AVFoundation

class VoxelSlice {
    var voxels:     [Voxel]
    let dimensions: SCNVector3
    let resolution: CGFloat
    
    init(dimensions: SCNVector3, resolution: CGFloat) {
        self.dimensions = dimensions
        self.resolution = resolution
        voxels          = Array(repeating: Voxel(),
                                count: Int(dimensions.x * dimensions.y * dimensions.z))
    }
    
    func getVoxelID(x: Int, y: Int, z: Int) -> Int {
        return (z * Int(dimensions.y) + y) * Int(dimensions.x) + x
    }
    
}
