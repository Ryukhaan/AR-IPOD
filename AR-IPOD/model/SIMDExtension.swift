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
}

extension Point3D {
    /**
     * Creates a unique Int from a Point3D (int, int, int)
     */
    func index() -> Int {
        let g = { (x: Int, y: Int) -> Int in
            let x1 = x + y
            return x1 * (x1 + 1) / 2 + y
            
        }
        return g(g(Int(x), Int(y)), Int(z))
    }
    
    /**
     * Get Point3D from a unique Int
     */
    static func inverse(n: Int) -> Point3D {
        let g_inv = { (n: Int) -> (Int, Int) in
            var m = Int(floor(sqrt(2.0 * Double(n))))
            var y = 0
            while true {
                y = n - m * (m + 1) / 2
                if y >= 0 { break }
                m -= 1
            }
            let x = m - y
            return (x, y)
        }
        let (w, z) = g_inv(n)
        let (x, y) = g_inv(w)
        return Point3D(Float(x), Float(y), Float(z))
    }
}
