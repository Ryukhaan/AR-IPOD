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
    func distance(ohter: Vector) -> Float {
        return (self - other).length()
    }
}
