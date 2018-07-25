//
//  KinectDepthImage.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 13/06/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit
import AVFoundation

class KinectDepthImage : DepthImage {
    
    override init() {
        super.init()
        width = Constant.Kinect.Width
        height = Constant.Kinect.Height
        data = [Float](repeating: 0.0, count: width * height)
    }
    
    override func switchTo(type: CameraType) -> DepthImage {
        switch type {
        case .iPhoneX:
            return IphoneDepthImage()
        default:
            return self
        }
    }
}
