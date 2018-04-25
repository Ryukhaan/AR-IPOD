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

class PointCloud {
    var vertices: [Vector] = [Vector]()
    var edges:    [Line]   = [Line]()
    
    func addPoint(point: Vector) {
        vertices.append(point)
    }
    func addEdge(edge: Line) {
        edges.append(edge)
    }
    
    func clear() {
        vertices.removeAll()
        edges.removeAll()
    }
    
    
}
