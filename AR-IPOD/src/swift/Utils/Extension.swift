//
//  Extension.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 05/06/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation

extension Vector {
    static func +(scalar: Float, vector: Vector) -> Vector {
        return Vector(scalar + vector.x, scalar + vector.y, scalar + vector.z)
    }
    
    static func +(vector: Vector, scalar: Float) -> Vector {
        return scalar + vector
    }
    
    static func -(scalar: Float, vector: Vector) -> Vector {
        return Vector(scalar - vector.x, scalar - vector.y, scalar - vector.z)
    }
    
    static func -(vector: Vector, scalar: Float) -> Vector {
        return scalar - vector
    }
}
