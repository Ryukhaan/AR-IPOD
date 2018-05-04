//
//  SIMDExtension.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 20/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

extension Vector {
    
    /**
     * Calculate vector's length (Euclidean nomr)
     */
    /*
    @inline(__always)
    func length() -> Float {
        return sqrtf(x*x + y*y + z*z)
    }
    */
    /**
     * Normalizes the vector  to length 1.0
     * Returns a new Vector.
     */
    /*
    @inline(__always)
    func normalized() -> Vector {
        return self / length()
    }
    */
    /**
     * Cacultates the distance between this Vector and an other one.
     */
    /*
    @inline(__always)
    func distance(other: Vector) -> Float {
        return (self - other).length()
    }
    */
    
    /*
    static func < (u: Vector, v: Vector) -> Bool {
        if u.x < v.x { return true }
        else if u.x > v.x { return false }
        if u.y < v.y { return true }
        else if u.y > v.y { return false }
        if u.z < v.z { return true }
        else if u.z > v.z { return false }
        return false
    }
    */
}
