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
    
    var intrinsics: matrix_float3x3 // Matrix K (state-of-the-art). Converts 3D point to 2D
    var extrinsics: matrix_float4x3 // Extrinsics camera : rotation and camera's position
    
    init(onRealTime: Bool) {
        if onRealTime {
            width = UInt16(Constant.IphoneWidth)
            height = UInt16(Constant.IphoneHeight)
        }
        else {
            width = UInt16(Constant.KinectWidth)
            height = UInt16(Constant.KinectHeight)
        }
        intrinsics = matrix_float3x3()
        extrinsics = matrix_float4x3(diagonal: float3(1,1,1))
    }
    
    /**
     * Initilialize camera given K and camera's dimensions
     */
    init(_intrinsics: matrix_float3x3, dim: CGSize) {
        intrinsics  = _intrinsics
        width       = UInt16(dim.width)
        height      = UInt16(dim.height)
        extrinsics  = matrix_float4x3()
    }
    
    mutating func update(intrinsics: matrix_float3x3) {
        self.intrinsics = intrinsics
    }
    /**
     * Updates extrinsics matrix (rotation and camera's position).
     */
    mutating func update(extrinsics: matrix_float4x4) {
        let Rt = extrinsics.transpose
        let newRt = matrix_float4x3(rows: [Rt.columns.0, Rt.columns.1, Rt.columns.2])
        //self.extrinsics = newRt
        self.extrinsics = newRt
    }
    
    mutating func changeTo(realTime: Bool) {
        if realTime {
            width = UInt16(Constant.IphoneWidth)
            height = UInt16(Constant.IphoneHeight)
        }
        else {
            width = UInt16(Constant.KinectWidth)
            height = UInt16(Constant.KinectHeight)
        }
    }
}
