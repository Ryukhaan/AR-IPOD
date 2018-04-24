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
    static let INVALID: CGFloat = 0.0   // Depth value for invalid pixel (normally isNaN ?)
    
    let width:  Int         // Image width  (IPhoneX : 360 pixel)
    let heigth: Int         // Image height (IPhoneX : 480 pixel)
    var data:   [CGFloat]   // Stores all depths in an array
    
    init(_width: Int, _heigth: Int) {
        width   = _width
        heigth  = _heigth
        data    = Array(repeating: 0.0, count: width*heigth)
    }
    
    /**
     * Returns depth at index : row * width + column.
     */
    func at(row: Int, column: Int) -> CGFloat {
        return data[row * Int(width) + column]
    }
    
    /**
     * Returns number of pixels in the image.
     */
    func numberOfPixels() -> Int {
        return width * heigth
    }
    
    /**
     * Updates depths values stored in data.
     */
    mutating func update(_data: UnsafeMutablePointer<CGFloat>) {
        for i in 0..<heigth {
            for j in 0..<width {
                data[i*width+j] = _data[i*width+j]
            }
        }
    }
    
    /**
     * Collects some statistics about the image (minimum, maximum and mean values).
     */
    func getStats() -> (CGFloat, CGFloat, CGFloat) {
        var minimum: CGFloat = 9999.0
        var maximum: CGFloat = -9999.0
        var mean: CGFloat    = 0.0
        for pixel in data {
            if pixel.isNaN { continue }
            minimum = min(minimum, pixel)
            maximum = max(maximum, pixel)
            mean += pixel
        }
        mean /= CGFloat(self.numberOfPixels())
        return (minimum, maximum, mean)
    }
}

