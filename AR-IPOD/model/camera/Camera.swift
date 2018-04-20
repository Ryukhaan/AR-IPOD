//
//  Camera.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 19/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit
import AVFoundation

struct Camera {
    let intrinsics: matrix_float3x3
    var extrinsics: matrix_float4x4

    let width:      UInt16
    let height:     UInt16
    init(intrinsics: matrix_float3x3, dimensions: CGSize) {
        self.intrinsics = intrinsics
        extrinsics      = matrix_float4x4()
        width           = UInt16(dimensions.width)
        height          = UInt16(dimensions.height)
    }
    
    mutating func update(position: matrix_float4x4) {
        extrinsics = position
    }
    
    func project(point: Vector) -> Vector {
        return intrinsics * (( 1.0 / point.z) * point)
    }
    
    func unproject(i: Int, j: Int, depth: Float) -> Vector {
        let homogene    = Vector(depth*Float(i), depth*Float(j), depth)
        return intrinsics.inverse * homogene
        
    }
}
