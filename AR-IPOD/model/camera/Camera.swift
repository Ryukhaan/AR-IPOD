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

    var width:      CGFloat = 0
    var height:     CGFloat = 0
    var zFar:       CGFloat = 0.0
    var zNear:      CGFloat = 0.0
    
    init() {
        intrinsics = matrix_float3x3(diagonal: Vector(1,1,1))
        extrinsics = matrix_float4x4(diagonal: float4(1,1,1,1))
    }
    
    init(_intrinsics: matrix_float3x3, dim: CGSize) {
        intrinsics  = _intrinsics
        extrinsics  = matrix_float4x4()
        width       = dim.width
        height      = dim.height
    }
    
    /**
     * Updates extrinsics matrix (rotation and position matrix).
     */
    mutating func update(position: matrix_float4x4) {
        extrinsics = position
    }
    
    func project(vector: Vector) -> Pixel {
        return AR_IPOD.project(vector: vector, K: intrinsics)
    }
    
    func unproject(i: Int, j: Int, depth: Float) -> Vector {
        return AR_IPOD.unproject(vector: Vector(Float(i), Float(j), depth), K: intrinsics)
    }
    
    func unproject(pixel: Pixel, depth: Float) -> Vector {
        return AR_IPOD.unproject(pixel: pixel, depth: depth, K: intrinsics)
    }
}
