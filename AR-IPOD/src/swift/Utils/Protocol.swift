//
//  CameraProtocol.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 13/06/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit
import AVFoundation

protocol CameraProtocol {
    func update(intrinsics: matrix_float4x4)
    func update(rotation: matrix_float3x3)
    func update(translation: vector_float3)
    func switchTo(type: CameraType) -> Camera
}

protocol DepthImageProtocol {
    func switchTo(type: CameraType) -> DepthImage
}
