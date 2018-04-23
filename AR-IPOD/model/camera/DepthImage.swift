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

struct DepthImage {
    static let INVALID: CGFloat = 0.0
    
    let width:  Int
    let heigth: Int
    var data:   [CGFloat]
    
    init(_width: Int, _heigth: Int) {
        width   = _width
        heigth  = _heigth
        data    = Array(repeating: 0.0, count: width*heigth)
    }
    
    func at(row: Int, column: Int) -> CGFloat {
        return data[row * Int(width) + column]
    }
    
    func numberOfPixels() -> Int {
        return width * heigth
    }
    
    mutating func update(_data: [CGFloat]) {
        assert(data.count == _data.count,
               "Data sizes are different")
        data = _data
    }
    
    func getStats() -> (CGFloat, CGFloat, CGFloat) {
        var minimum: CGFloat = 9999.0
        var maximum: CGFloat = -9999.0
        var mean: CGFloat    = 0.0
        let numberOfPixels   = self.numberOfPixels()
        for i in 0..<numberOfPixels {
            let pixel = data[Int(i)]
            if pixel == DepthImage.INVALID { continue }
            if pixel.isNaN { continue }
            minimum = min(minimum, pixel)
            maximum = max(maximum, pixel)
            mean += pixel
        }
        mean /= CGFloat(numberOfPixels)
        return (minimum, maximum, mean)
    }
}

