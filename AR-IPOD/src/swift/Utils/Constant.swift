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
    enum Code {
        enum Integration {
            static let hasFinished: String = "200"
            static let isStarting: String = "100"
            static let error:   String  = "400"
            static let reset:   String  = "600"
        }
        enum Photo {
            static let sendingPhoto: String = "110"
        }
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
    case iPhoneX
    case Other
}

enum DeviceType {
    case iPhoneX
    case iPad
}
