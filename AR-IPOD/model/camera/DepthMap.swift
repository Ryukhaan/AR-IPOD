//
//  Depthmap.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 19/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit
import AVFoundation

struct DepthMap {
    let width:  UInt16
    let heigth: UInt16
    var data:   [Float]
    
    init(width: UInt16, heigth: UInt16) {
        self.width  = width
        self.heigth = heigth
        data        = Array(repeating: 0.0, count: Int(width*heigth))
    }
    
    func at(row: Int, column: Int) -> Int {
        return row * Int(width) + column
    }
    
    mutating func update(data: [Float]) {
        assert(self.data.count == data.count,
               "Data sizes are different")
        self.data = data
    }
}

