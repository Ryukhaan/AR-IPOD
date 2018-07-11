//
//  Constant.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 05/06/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation

enum Constant {
    enum Kinect {
        static let Width    = 640
        static let Height   = 480
    }
    enum Iphone {
        static let Width    = 360
        static let Height   = 640
    }
}

enum Sounds {
    static let beep = Bundle.main.path(forResource: "beep", ofType: "wav")
}

enum AcquisitionType {
    case RealTime
    case Ikea
    case Chair
}

enum CameraType {
    case Kinect
    case Iphone
    case Other
}

enum DeviceType {
    case Iphone
    case IPad
}
