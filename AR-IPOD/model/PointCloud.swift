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
    var points: [SCNVector3]
    
    mutating func addPoint(point: SCNVector3) {
        self.points.append(point)
    }
    
    mutating func clear() {
        self.points.removeAll()
    }
}
