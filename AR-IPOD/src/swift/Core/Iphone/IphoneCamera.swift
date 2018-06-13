//
//  IphoneCamera.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 13/06/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit
import AVFoundation

class IphoneCamera: Camera {
    override init() {
        super.init()
        width   = Constant.Iphone.Width
        height  = Constant.Iphone.Height
    }
    
    init(from: Camera) {
        super.init()
        width   = Constant.Iphone.Width
        height  = Constant.Iphone.Height
        intrinsics  = from.intrinsics
        rotation    = from.rotation
        translation = from.translation
    }
    
    override func update(intrinsics: matrix_float4x4) {
        self.intrinsics = intrinsics
    }
    override func update(rotation: matrix_float3x3) {
        self.rotation = rotation
    }
    override func update(translation: vector_float3) {
        self.translation = translation
    }
    
    override func switchTo(type: CameraType) -> Camera {
        switch type {
        case .Kinect:
            return KinectCamera(from: self)
        default:
            return self;
        }
    }
}
