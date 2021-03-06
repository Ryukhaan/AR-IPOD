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
    func length() -> Float {
        return sqrtf(x*x + y*y + z*z)
    }
    
    /**
     * Normalizes the vector  to length 1.0
     * Returns a new Vector.
     */
    func normalized() -> Vector {
        return self / length()
    }
    
    /**
     * Cacultates the distance between this Vector and an other one.
     */
    func distance(other: Vector) -> Float {
        return (self - other).length()
    }
    
    static func < (u: Vector, v: Vector) -> Bool {
        if u.x < v.x { return true }
        else if u.x > v.x { return false }
        if u.y < v.y { return true }
        else if u.y > v.y { return false }
        if u.z < v.z { return true }
        else if u.z > v.z { return false }
        return false
    }
}

extension Point3D {
    
    static func + (scalar: Float, vector: Point3D) -> Vector {
        return Vector(scalar + vector.x, scalar + vector.y, scalar + vector.z)
    }
    /**
     * Creates a unique Int from a Point3D (int, int, int)
     */
    func index(base: Int) -> Int {
        let a: Int = Int(x) * (base * base)
        let b: Int = Int(y) * base
        let c: Int = Int(z)
        return a + b + c
    }
    
    /**
     * Get Point3D from a unique Int
     */
    static func hashInverse(n: Int, base: Int) -> Point3D {
        let fbase: Float = Float(base)
        var remainder: Float = Float(n)
        let x: Float = floor(remainder / (fbase * fbase))
        remainder = remainder - x * (fbase * fbase)
        let y: Float = floor(remainder / Float(base))
        remainder = remainder - x * fbase
        let z: Float = remainder
        return Point3D(Float(x), Float(y), Float(z))
    }
}
