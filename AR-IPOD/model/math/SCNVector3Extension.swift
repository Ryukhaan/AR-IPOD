//
//  SCNVector3Extension.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 20/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

extension SCNVector3
{
    /**
    * Adds two SCNVector3
    */
    static func + (left: SCNVector3, right: SCNVector3) -> SCNVector3 {
        return SCNVector3Make(left.x + right.x, left.y + right.y, left.z + right.z)
    }
    
    /**
     * Substracts two SCNVector3
     */
    static func - (left: SCNVector3, right: SCNVector3) -> SCNVector3 {
        return SCNVector3Make(left.x - right.x, left.y - right.y, left.z - right.z)
    }
    
    /**
     * Multiplies, element-wise, two SCNVector3
     */
    static func * (left: SCNVector3, right: SCNVector3) -> SCNVector3 {
        return SCNVector3Make(left.x * right.x, left.y * right.y, left.z * right.z)
    }
    
    /**
    * Multiplies a SCNVector3 by a Float.
    */
    static func * (scalar: Float, vector: SCNVector3) -> SCNVector3 {
        return SCNVector3Make(scalar * vector.x, scalar * vector.y, scalar * vector.z)
    }
    
    /**
     * Divides a SCNVector by a Float.
     */
    static func / (vector: SCNVector3, scalar: Float) -> SCNVector3 {
        return SCNVector3Make(vector.x / scalar, vector.y / scalar, vector.z / scalar)
    }
    
    /**
     * Computes cross product between two SCNVector3.
     */
    static func cross(u: SCNVector3, v: SCNVector3) -> SCNVector3 {
        return SCNVector3Make(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x)
    }
    
    /**
     * Computes dot product between two SCNVector3.
     */
    static func dot(u: SCNVector3, v: SCNVector3) -> Float {
        return u.x * v.x + u.y * v.y + u.z * v.z
    }
    
    /**
     * Returns the length (magnitude) of the vector described by the SCNVector3
     */
    func length() -> Float {
        return sqrtf(x*x + y*y + z*z)
    }
    
    /**
     * Normalizes the vector described by the SCNVector3 to length 1.0 and returns
     * the result as a new SCNVector3.
     */
    func normalized() -> SCNVector3 {
        return self / length()
    }
    
    /**
     * Normalizes the vector described by the SCNVector3 to length 1.0.
     */
    mutating func normalize() -> SCNVector3 {
        self = normalized()
        return self
    }
    
    /**
     * Cacultates the distance between two SCNVector3.
     */
    func distance(vector: SCNVector3) -> Float {
        return (self - vector).length()
    }
}

