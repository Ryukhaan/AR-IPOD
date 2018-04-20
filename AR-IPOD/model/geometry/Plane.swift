//
//  Plane.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 20/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

class Plane {
    var normal:     SCNVector3
    var distance:   Float
    
    init(vector: SCNVector3, d: Float) {
        normal      = vector
        distance    = d
    }
    init(vector: SCNVector4) {
        normal      = SCNVector3Make(vector.x, vector.y, vector.z)
        distance    = vector.w
    }
    init(_ a: SCNVector3, _ b:SCNVector3, _ c:SCNVector3) {
        normal   = SCNVector3.cross(u: b-a, v: c-a)
        distance = 0
        //distance = distance(vector: normal)
    }
}
