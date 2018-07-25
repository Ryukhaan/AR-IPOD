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
    var collectionSize: Int     = 6
    var datasCollection: [[Float]]    = [[Float]]() // Stores all
    
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
        OperationQueue.main.addOperation {
            let n = self.numberOfPixels()
            var s = self.datasCollection.count
            if s >= self.collectionSize {
                self.datasCollection.remove(at: 0)
            }
            self.datasCollection.append([Float](repeating: 0.0, count: n))
            s = self.datasCollection.count
            for i in 0..<self.height {
                for j in 0..<self.width {
                    self.datasCollection[s-1][i*self.width+j] = map[i*self.width+j]
                }
            }
        }
    }
    
    func collect() {
        for i in 0..<height {
            for j in 0..<width {
                let index = i*width+j
                var array = [Float](repeating: 0.0, count: collectionSize)
                for k in 0..<collectionSize {
                    array.append(datasCollection[k][index])
                }
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
        case .iPhoneX:
            return IphoneDepthImage()
        case .Kinect:
            return KinectDepthImage()
        default:
            return self
        }
    }
}

