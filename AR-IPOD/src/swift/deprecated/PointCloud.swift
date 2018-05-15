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
    
    @inline(__always)
    func addPoint(point: Vector) {
        vertices.append(point)
    }
    
    @inline(__always)
    func addEdge(edge: Line) {
        edges.append(edge)
    }
    
    @inline(__always)
    func clear() {
        vertices.removeAll()
        edges.removeAll()
    }
    
    
}
