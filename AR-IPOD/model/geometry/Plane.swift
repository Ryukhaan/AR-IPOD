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
    var normal:     Vector
    var distance:   Float
    
    init(vector: Vector, d: Float) {
        normal      = vector
        distance    = d
    }
    init(vector: float4) {
        normal      = Vector(vector.x, vector.y, vector.z)
        distance    = vector.w
    }
    init(_ a: Vector, _ b:Vector, _ c:Vector) {
        normal   = cross(b-a, c-a)
        distance = 0
        //distance = distance(vector: normal)
    }
}
