//
//  VoxelSlice.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 19/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit
import AVFoundation

class Volume {
    // Singleton pattern : Only one volume will be create
    static let sharedInstance = Volume()
    
    var resolutionInMeter: Float   // Number of voxels per meter
    // Can size be only an Int ? Let's do this !
    //var size:       Point3D // X size, Y size and Z size
    var numberOfVoxels:   Int
    
    // Since i now the hash function, is adictionary still necessary ?
    //lazy var voxels:     [Int: Voxel]   = self.allocate()
    //lazy var centroids:  [Int: Vector]  = self.allocate()
    lazy var voxels:    [Voxel]     = self.allocate()
    lazy var centroids: [Vector]    = self.allocate()
    //lazy var centroids: [Id3: Vector] = [Int: Vector]()
    
    private init() {
        numberOfVoxels      = 64
        resolutionInMeter   = 4.0
    }
    
    func allocate<T>() -> [T] {
        var allocator = [T]()
        allocator.removeAll()
        allocator.reserveCapacity(totalOfVoxels())
        return allocator
    }
    
    func initialize() {
        /* Sequence - serial */
        let count = totalOfVoxels()
        //let square = size * size
        voxels = [Voxel](repeating: Voxel(), count: count)
        //centroids = [Vector](repeating: Point3D(0, 0, 0), count: count)
        
        let stride = MemoryLayout<Point3D>.stride
        let byteCount = stride * count
        let points = UnsafeMutablePointer<Point3D>.allocate(capacity: byteCount)
        bridge_initializeCentroids(points, Int32(numberOfVoxels), resolutionInMeter)
        let buffer = UnsafeBufferPointer(start: points, count: count)
        centroids = Array(buffer)
        points.deallocate()
    }
    
    func initialize(with: Int) {
        numberOfVoxels = with
        initialize()
    }
    
    func totalOfVoxels() -> Int {
        return numberOfVoxels * numberOfVoxels * numberOfVoxels
    }
    
    func integrateDepthMap(image: DepthImage, camera: inout Camera, parameters: [Float]) {
        let width = image.width
        let height = image.height
        let dethmap = image.data
        let resolve = [resolutionInMeter, resolutionInMeter, resolutionInMeter]
        _ = bridge_integrateDepthMap(dethmap, centroids, &camera.extrinsics, &camera.intrinsics, &voxels, Int32(width), Int32(height), Int32(numberOfVoxels), resolve, parameters[0], parameters[1]);
        //let voxel2 = voxels[numberOfVoxels].sdf
    }
}
