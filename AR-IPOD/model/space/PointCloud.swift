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
    var data: [Vector] = [Vector]()
    
    mutating func addPoint(point: Vector) {
        data.append(point)
    }
    
    mutating func clear() {
        data.removeAll()
    }
}
