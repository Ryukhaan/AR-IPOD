//
//  Types.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 20/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

typealias Vector    = float3
typealias Point3D   = float3
typealias Line      = int2
typealias Id3       = int3
typealias Pixel     = float2

struct Constant {
    static let KinectWidth = 640
    static let KinectHeight = 480
    
    static let IphoneWidth = 360
    static let IphoneHeight = 480
}
