//
//  Extension.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 05/06/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

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
    
    init(_ v : float4) {
        self.init()
        self.x = v.x
        self.y = v.y
        self.z = v.z
    }
    
    init(_ v: int3) {
        self.init()
        self.x = Float(v.x)
        self.y = Float(v.y)
        self.z = Float(v.z)
    }
    
    init(_ v: m_float3) {
        self.init()
        self.x = v.x
        self.y = v.y
        self.z = v.z
    }
}

extension Float {
    func format(_ f: String) -> String {
        return String(format: "%\(f)f", self)
    }
}


extension matrix_float4x4 {
    func format(_ f: String) -> matrix_float4x4 {
        var res = matrix_float4x4(diagonal: float4(1,1,1,1))
        res.columns.0 = float4(Float(Float(res.columns.0.x).format(f))!,
                               Float(Float(res.columns.0.y).format(f))!,
                               Float(Float(res.columns.0.z).format(f))!,
                               Float(Float(res.columns.0.w).format(f))!)
        res.columns.1 = float4(Float(Float(res.columns.1.x).format(f))!,
                               Float(Float(res.columns.1.y).format(f))!,
                               Float(Float(res.columns.1.z).format(f))!,
                               Float(Float(res.columns.1.w).format(f))!)

        res.columns.2 = float4(Float(Float(res.columns.2.x).format(f))!,
                               Float(Float(res.columns.2.y).format(f))!,
                               Float(Float(res.columns.2.z).format(f))!,
                               Float(Float(res.columns.2.w).format(f))!)

        res.columns.3 = float4(Float(Float(res.columns.3.x).format(f))!,
                               Float(Float(res.columns.3.y).format(f))!,
                               Float(Float(res.columns.3.z).format(f))!,
                               Float(Float(res.columns.3.w).format(f))!)
        return res
    }
}

extension float4 {
    init(_ v : float3, _ x: Float) {
        self.init()
        self.x = v.x
        self.y = v.y
        self.z = v.z
        self.w = x
    }
    
    init(_ v: float3) {
        self.init(v, 1.0)
    }
}
