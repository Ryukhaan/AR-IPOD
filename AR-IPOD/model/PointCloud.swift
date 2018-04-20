//
//  PointCloud.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 19/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit
import AVFoundation

struct PointCloud {
    var data: [float3]
    
    mutating func addPoint(point: float3) {
        data.append(point)
    }
    
    mutating func clear() {
        data.removeAll()
    }
}
