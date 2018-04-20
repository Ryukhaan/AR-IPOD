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
    func length() -> Float {
        return sqrtf(x*x + y*y + z*z)
    }
    
    /**
     * Normalizes the vector described by the Vector to length 1.0 and returns
     * the result as a new Vector.
     */
    func normalized() -> Vector {
        return self / length()
    }
    
    /**
     * Cacultates the distance between two SCNVector3.
     */
    func distance(vector: Vector) -> Float {
        return (self - vector).length()
    }
    
}
