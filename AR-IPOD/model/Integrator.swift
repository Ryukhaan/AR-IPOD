//
//  Integrator.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 20/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

func integratePointCloud(pointCloud: inout PointCloud,
                         camera: Camera,
                         maxDist: Float) {
    // TO DO : modulate / Slice manager
    let roundToVoxel: Float = 490.0
    let startCamera         = float3(camera.extrinsics[0][3],
                                     camera.extrinsics[1][3],
                                     camera.extrinsics[2][3])
    let min = int3(-9999, -9999, -9999)
    let max = int3(9999, 9999, 999)
    for point in pointCloud.data {
        var raycastVoxels   = [int3]()
        let worldPoint      = camera.intrinsics * point
        let depth           = point.z
        
        // Too near
        if (depth < 0.01) { continue }
        
        let direction   = (worldPoint - startCamera).normalized()
        // Need a class to be modulate
        let truncation: Float   = 50
        let start               = roundToVoxel * startCamera
        let end                 = roundToVoxel * (worldPoint + direction * truncation)
        
        //raycastVoxels.removeAll()
        raycast(start: start, end: end, min: min, max: max, output: &raycastVoxels)
        assert(raycastVoxels.count < 500, "Too many raycast voxels")
        
        for voxel in raycastVoxels {
            // TO DO
            continue
        }
    }
}
