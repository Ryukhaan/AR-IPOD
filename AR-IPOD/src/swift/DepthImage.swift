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
    var width:  UInt16          // Image width  (IPhoneX : 360 pixel)
    var height: UInt16          // Image height (IPhoneX : 480 pixel)
    var data:   [Float]         // Stores all depths in an array
    var savedData: [[Float]]    = [[Float]]()
    
    init(onRealTime: Bool) {
        if onRealTime {
            width = UInt16(Constant.IphoneWidth)
            height = UInt16(Constant.IphoneHeight)
        }
        else {
            width = UInt16(Constant.KinectWidth)
            height = UInt16(Constant.KinectHeight)
        }
        data = Array<Float>(repeating: 0, count: Int(width)*Int(height))
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
        // Need to convert width and height to Int otherwise overflow then segment fault (or something like that)
        return Int(width) * Int(height)
    }
    
    /**
     * Updates depths values stored in data.
     */
    mutating func update(_data: UnsafeMutablePointer<Float>) {
        let countH = Int(height)
        let countW = Int(width)
        for i in 0..<countH {
            for j in 0..<countW {
                data[i*countW+j] = _data[i*countW+j]
            }
        }
    }
    
    mutating func push(map: UnsafeMutablePointer<Float>) {
        let countH = Int(height)
        let countW = Int(width)
        if savedData.count >= 6 {
            savedData.remove(at: 0)
        }
        savedData.append([Float](repeating: 0.0, count: countH*countW))
        for i in 0..<countH {
            for j in 0..<countW {
                savedData[savedData.count-1][i*countW+j] = map[i*countW+j]
            }
        }
    }
    
    mutating func updateDataWithSavedData() {
        let countH = Int(height)
        let countW = Int(width)
        for i in 0..<countH {
            for j in 0..<countW {
                let index = i*countW+j
                let array  = [savedData[0][index],
                               savedData[1][index],
                               savedData[2][index],
                               savedData[3][index],
                               savedData[4][index],
                               savedData[5][index]]
                let sorted = array.sorted()
                if sorted.count % 2 == 0 {
                    data[index] = Float((sorted[(sorted.count / 2)] + sorted[(sorted.count / 2) - 1])) / 2
                }
                else {
                    data[index] = Float(sorted[(sorted.count - 1) / 2])
                }
            }
        }
    }
    /**
     * Updates depths values stored in UInt8 Array.
     */
    mutating func update(_data: [UInt8]) {
        data = [Float](repeating: 0, count: numberOfPixels())
        let countH = Int(height)
        let countW = Int(width)
        for i in 0..<countH {
            for j in 0..<countW {
                data[i*countW+j] = Float(_data[(i*countW+j) % _data.count])
            }
        }
    }
    
    /**
     * Updates depths values given an array
     */
    mutating func update(_data: [Float]) {
        assert(data.count == _data.count, "Dimension are not equals ! \(data.count) vs \(_data.count)")
        data = _data
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
        data = Array<Float>(repeating: 0, count: Int(width)*Int(height))
    }
    
    /**
     * Collects some statistics about the image (minimum, maximum and mean values).
     */
    /*
    func getStats() -> (Float, Float, Float) {
        var minimum: Float = 9999.0
        var maximum: Float = -9999.0
        var mean: Float    = 0.0
        for pixel in self.data {
            if pixel.isNaN { continue }
            minimum = min(minimum, pixel)
            maximum = max(maximum, pixel)
            mean += pixel
        }
        mean /= Float(self.numberOfPixels())
        return (minimum, maximum, mean)
    }
    */
}

