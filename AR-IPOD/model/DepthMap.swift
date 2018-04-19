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
    let resolution: CGSize
    var data:       [CGFloat]
    
    init(resolution: CGSize) {
        self.resolution = resolution
        data = Array(repeating: 0.0, count: Int(resolution.width * resolution.height))
    }
    
    init(width: Int, heigth: Int) {
        self.resolution = CGSize(width: width, height: heigth)
        data = Array(repeating: 0.0, count: width*heigth)
    }
    
    func getIndex(row: Int, column: Int, width: Int) -> Int {
        return row * width + column
    }
    
    func getWidth(from: DepthMap) -> Int {
        return Int(from.resolution.width)
    }
    
    func getHeight(from: DepthMap) -> Int {
        return Int(from.resolution.height)
    }
    
    func atIndex(from: DepthMap, i: Int, j: Int) -> CGFloat {
        let w = getWidth(from: from)
        return from.data[getIndex(row: i, column: j, width: w)]
    }
    
    mutating func update(data: [CGFloat]) {
        assert(self.data.count == data.count,
               "Data sizes are different")
        self.data = data
    }
}

