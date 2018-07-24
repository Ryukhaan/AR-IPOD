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

/**
 * Camera stores all information about camera and
 * mathematics' objects needed to convert 3D point to 2D (or 2D to 3D).
 * It stores all information provided by VIO and tracking feature given by
 * ARFaceTrackingConfiguration.
 * See ARCamera for more information and details, since Camera is an summary of ARCamera.
 */
class Camera : CameraProtocol {
    var height:     Int = 0     // Camera height - 480 IphoneX
    var width:      Int = 0     // Camera width  - 360 IphoneX
    var zFar:       Float   = 0.0   // Nearest point seen by camera
    var zNear:      Float   = 0.0   // Farther point ssen by camera
    
    var intrinsics:     matrix_float4x4 = matrix_float4x4(diagonal: float4(1,1,1,1)) // Matrix K
    var rotation:       matrix_float3x3 = matrix_float3x3(diagonal: float3(1,1,1)) // Camera's rotation
    var translation:    Vector = Vector(0,0,0) // Camera's translation
    var rotationLie:    Vector = Vector(-9999,-9999,-9999)
    
    func update(intrinsics: matrix_float4x4) {}
    func update(rotation: matrix_float3x3) {}
    func update(translation: vector_float3) {}
    func switchTo(type: CameraType) -> Camera {
        switch type {
        case .Iphone:
            return IphoneCamera(from: self)
        case .Kinect:
            return KinectCamera(from: self)
        default:
            return self
        }
    }
}
