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
    let dimensions: CGSize
    
    init(intrinsics: matrix_float3x3, dimensions: CGSize) {
        self.intrinsics = intrinsics
        extrinsics      = matrix_float4x4()
        self.dimensions = dimensions
    }
    
    mutating func update(position: matrix_float4x4) {
        extrinsics = position
    }
    
    func project(point: float3) -> float3 {
        return intrinsics * (( 1.0 / point.z) * point)
    }
    
    func unproject(pixel: float2, depth: Float) -> float3 {
        let homogene    = float3(depth*pixel.x, depth*pixel.y, depth)
        return intrinsics.inverse * homogene
        
    }
}

