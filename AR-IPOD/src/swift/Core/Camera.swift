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
struct Camera {
    var height:     UInt16     // Camera height - 480 IphoneX
    var width:      UInt16     // Camera width  - 360 IphoneX
    var zFar:       Float   = 0.0   // Nearest point seen by camera
    var zNear:      Float   = 0.0   // Farther point ssen by camera
    
    var intrinsics: matrix_float4x4 // Matrix K (state-of-the-art). Converts 3D point to 2D
    var extrinsics: matrix_float4x4 // Extrinsics camera : rotation and camera's position
    
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
        extrinsics = matrix_float4x4(diagonal: float4(1,1,1,1))
    }
    
    mutating func update(intrinsics: matrix_float3x3) {
        self.intrinsics = matrix_float4x4(diagonal: float4(1,1,1,1))
        // First column
        self.intrinsics.columns.0.x = intrinsics.columns.0.x
        self.intrinsics.columns.0.z = intrinsics.columns.0.y
        self.intrinsics.columns.0.y = intrinsics.columns.0.z
        // Second column
        self.intrinsics.columns.0.x = intrinsics.columns.1.x
        self.intrinsics.columns.1.z = intrinsics.columns.1.y
        self.intrinsics.columns.2.y = intrinsics.columns.1.z
        // Third column
        self.intrinsics.columns.0.x = intrinsics.columns.2.x
        self.intrinsics.columns.1.z = intrinsics.columns.2.y
        self.intrinsics.columns.2.y = intrinsics.columns.2.z
    }
    
    mutating func update(intrinsics: matrix_float4x4) {
        self.intrinsics = intrinsics
    }
    
    /**
     * Updates extrinsics matrix (rotation and camera's position).
     */
    mutating func update(extrinsics: matrix_float4x4) {
        self.extrinsics = extrinsics
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
