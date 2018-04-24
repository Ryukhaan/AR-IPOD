//
//  Plane.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 20/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

/**
 * Plane stands for the mathematic object called Plane !
 */
class Plane {
    var normal:     Vector  // Normal
    var distance:   Float   // ???
    
    init() {
        normal      = Vector(0, 0, 0)
        distance    = 0.0
    }
    
    /**
     * Initializes a plane given a Vector and a distance
     */
    init(vector: Vector, d: Float) {
        normal      = vector
        distance    = d
    }
    
    /**
     * Initializes a plane given a float4. First three values contains normals values.
     * Last one stands for the distance.
     */
    init(vector: float4) {
        normal      = Vector(vector.x, vector.y, vector.z)
        distance    = vector.w
    }
    
    /**
     * Initializes a plane given three vectors belonging to it.
     */
    init(_ a: Vector, _ b:Vector, _ c:Vector) {
        normal   = cross(b-a, c-a)
        distance = 0
        //distance = distance(vector: normal)
    }
}
