//
//  Extension.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 05/06/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation

extension Data {
    func copyBytes<T>(as _: T.Type) -> [T] {
        return withUnsafeBytes { (bytes: UnsafePointer<T>) in
            Array(UnsafeBufferPointer(start: bytes, count: count / MemoryLayout<T>.stride))
        }
    }
}

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
