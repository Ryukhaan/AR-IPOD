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
    
    var resolution: Float   // Number of voxels per meter
    // Can size be only an Int ? Let's do this !
    //var size:       Point3D // X size, Y size and Z size
    var size:   Int
    
    // Since i now the hash function, is adictionary still necessary ?
    //lazy var voxels:     [Int: Voxel]   = self.allocate()
    //lazy var centroids:  [Int: Vector]  = self.allocate()
    lazy var voxels:    [Voxel]     = self.allocate()
    lazy var centroids: [Vector]    = self.allocate()
    
    private init() {
        size        = 64
        resolution  = 1.0
    }
    
    func allocate<T>() -> [T] {
        var allocator = [T]()
        allocator.removeAll()
        allocator.reserveCapacity(numberOfVoxels())
        return allocator
    }
    
    func initialize() {
        /* Sequence - serial */
        let count = numberOfVoxels()
        //let square = size * size
        voxels = [Voxel](repeating: Voxel(), count: count)
        //centroids = [Vector](repeating: Point3D(0, 0, 0), count: count)
        
        let stride = MemoryLayout<Point3D>.stride
        let byteCount = stride * count
        let points = UnsafeMutablePointer<Point3D>.allocate(capacity: byteCount)
        bridge_initializeCentroids(points, Int32(size), resolution)
        let buffer = UnsafeBufferPointer(start: points, count: count)
        centroids = Array(buffer)
        points.deallocate()
    }
    
    func initialize(with: Int) {
        size = with
        initialize()
    }
    
    func numberOfVoxels() -> Int {
        return size * size * size
    }
    
    func integrateDepthMap(image: DepthImage, camera: inout Camera) {
        let width = image.width
        let height = image.height
        let dethmap = image.data
        let resolve = [resolution, resolution, resolution]
        _ = bridge_integrateDepthMap(dethmap, centroids, &camera.extrinsics, &camera.intrinsics, &voxels, Int32(width), Int32(height), Int32(size), resolve)
        let voxel2 = voxels[size].sdf
        
        
    }
}
