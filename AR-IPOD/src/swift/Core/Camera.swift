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

/*
struct Camera {
    var height:     UInt16     // Camera height - 480 IphoneX
    var width:      UInt16     // Camera width  - 360 IphoneX
    var zFar:       Float   = 0.0   // Nearest point seen by camera
    var zNear:      Float   = 0.0   // Farther point ssen by camera
    
    var intrinsics: matrix_float4x4 // Matrix K (state-of-the-art). Converts 3D point to 2D
    var rotation: matrix_float3x3  // Extrinsics camera : camera's rotation
    var translation: vector_float3 // Extrinsics camera : camera's translation
    
    init(onRealTime: Bool) {
        if onRealTime {
            width = UInt16(Constant.Iphone.Width)
            height = UInt16(Constant.Iphone.Height)
        }
        else {
            width = UInt16(Constant.Kinect.Width)
            height = UInt16(Constant.Kinect.Height)
        }
        intrinsics = matrix_float4x4(diagonal: float4(1,1,1,1))
        rotation   = matrix_float3x3(diagonal: float3(1,1,1))
        translation = float3(0,0,0)
    }
    
    mutating func update(intrinsics: matrix_float3x3) {
        self.intrinsics = matrix_float4x4(diagonal: float4(1,1,1,1))
        // First column
        self.intrinsics.columns.0.x = intrinsics.columns.0.x
        self.intrinsics.columns.0.y = intrinsics.columns.0.y
        self.intrinsics.columns.0.z = intrinsics.columns.0.z
        // Second column
        self.intrinsics.columns.1.x = intrinsics.columns.1.x
        self.intrinsics.columns.1.y = intrinsics.columns.1.y
        self.intrinsics.columns.1.z = intrinsics.columns.1.z
        // Third column
        self.intrinsics.columns.2.x = intrinsics.columns.2.x
        self.intrinsics.columns.2.y = intrinsics.columns.2.y
        self.intrinsics.columns.2.z = intrinsics.columns.2.z
    }
    
    mutating func update(intrinsics: matrix_float4x4) {
        self.intrinsics = intrinsics
    }
    
    /**
     * Updates extrinsics matrix (rotation and camera's position).
     */
    mutating func update(rotation: matrix_float3x3) {
        //self.extrinsics = matrix_float4x4(diagonal: float4(1,1,1,1))
        // Beware, there is no translation in IPhone (we do "fast_icp" instead)
        self.rotation.columns.0.x = 1
        self.rotation.columns.0.y = rotation.columns.0.y
        self.rotation.columns.0.z = rotation.columns.0.z
        // Second column
        self.rotation.columns.1.x = rotation.columns.1.x
        self.rotation.columns.1.y = 1
        self.rotation.columns.1.z = rotation.columns.1.z
        // Third column
        self.rotation.columns.2.x = rotation.columns.2.x
        self.rotation.columns.2.y = rotation.columns.2.y
        self.rotation.columns.2.z = 1
    }
    
    mutating func update(translation: vector_float3) {
        self.translation = translation
    }
    
    mutating func changeTo(realTime: Bool) {
        if realTime {
            width = UInt16(Constant.Iphone.Width)
            height = UInt16(Constant.Iphone.Height)
        }
        else {
            width = UInt16(Constant.Kinect.Width)
            height = UInt16(Constant.Kinect.Height)
        }
    }
    
}
 */
