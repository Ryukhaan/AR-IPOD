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

/**
 * DepthImage stores all information relative to depth map.
 */
struct DepthImage {    
    let width:  UInt16         // Image width  (IPhoneX : 360 pixel)
    let heigth: UInt16         // Image height (IPhoneX : 480 pixel)
    var data:   [Float]        // Stores all depths in an array
    
    init(_width: UInt16, _heigth: UInt16) {
        width   = _width
        heigth  = _heigth
        data    = [Float](repeating: 0, count: Int(width*heigth))
    }
    
    /**
     * Returns depth at index : row * width + column.
     */
    func at(row: Int, column: Int) -> Float {
        return data[row * Int(width) + column]
    }
    
    /**
     * Returns number of pixels in the image.
     */
    func numberOfPixels() -> Int {
        return Int(width * heigth)
    }
    
    /**
     * Updates depths values stored in data.
     */
    mutating func update(_data: UnsafeMutablePointer<Float>) {
        let countH = Int(heigth)
        let countW = Int(width)
        for i in 0..<countH {
            for j in 0..<countW {
                data[i*countW+j] = _data[i*countW+j]
            }
        }
    }
    
    /**
     * Collects some statistics about the image (minimum, maximum and mean values).
     */
    func getStats() -> (Float, Float, Float) {
        var minimum: Float = 9999.0
        var maximum: Float = -9999.0
        var mean: Float    = 0.0
        for pixel in data {
            if pixel.isNaN { continue }
            minimum = min(minimum, pixel)
            maximum = max(maximum, pixel)
            mean += pixel
        }
        mean /= Float(self.numberOfPixels())
        return (minimum, maximum, mean)
    }
}

