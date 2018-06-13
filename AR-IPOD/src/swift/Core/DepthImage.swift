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
class DepthImage : DepthImageProtocol {    
    var width:  Int             = 0// Image width  (IPhoneX : 360 pixel)
    var height: Int             = 0// Image height (IPhoneX : 480 pixel)
    var data:   [Float]         = [Float]()  // Stores all depths in an array
    var savedData: [[Float]]    = [[Float]]()
    
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
        return width * height
    }
    
    /**
     * Updates depths values stored in data.
     */
    func update(_data: UnsafeMutablePointer<Float>) {
        for i in 0..<height {
            for j in 0..<width {
                data[i*width+j] = _data[i*width+j]
            }
        }
    }
    
    func push(map: UnsafeMutablePointer<Float>) {
        let n = numberOfPixels()
        if savedData.count >= 6 {
            savedData.remove(at: 0)
        }
        savedData.append([Float](repeating: 0.0, count: n))
        for i in 0..<height {
            for j in 0..<width {
                savedData[savedData.count-1][i*width+j] = map[i*width+j]
            }
        }
    }
    
    func updateDataWithSavedData() {
        for i in 0..<height {
            for j in 0..<width {
                let index = i*width+j
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
        savedData.removeAll()
    }
    /**
     * Updates depths values stored in UInt8 Array.
     */
    func update(_data: [UInt8]) {
        data = [Float](repeating: 0, count: numberOfPixels())
        for i in 0..<height {
            for j in 0..<width {
                data[i*width+j] = Float(_data[(i*width+j) % _data.count])
            }
        }
    }
    
    /**
     * Updates depths values given an array
     */
    func update(_data: [Float]) {
        assert(data.count == _data.count, "Dimension are not equals ! \(data.count) vs \(_data.count)")
        data = _data
    }
    
    func switchTo(type: CameraType) -> DepthImage {
        switch type {
        case .Iphone:
            return IphoneDepthImage()
        case .Kinect:
            return KinectDepthImage()
        default:
            return self
        }
    }
    /*
     if realTime {
     width = UInt16(Constant.Iphone.Width)
     height = UInt16(Constant.Iphone.Height)
     }
     else {
     width = UInt16(Constant.Kinect.Width)
     height = UInt16(Constant.Kinect.Height)
     }
     data = Array<Float>(repeating: 0, count: Int(width)*Int(height))
     }
     */
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

