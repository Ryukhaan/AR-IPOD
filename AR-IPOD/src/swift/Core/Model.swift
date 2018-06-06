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

class Model {
    // Singleton pattern : Only one volume will be create
    static let sharedInstance = Model()
    
    var resolutionInMeter: Float
    var numberOfVoxels:   Int
    var voxels: [Voxel]
    var camera: Camera
    var image: DepthImage
    var parameters: [String: Float] = [String: Float]()
    
    private init() {
        numberOfVoxels      = 256
        resolutionInMeter   = 3.0
        voxels = [Voxel](repeating: Voxel(), count: Int(pow(Float(numberOfVoxels), 3.0)))
        camera = Camera(onRealTime: false)
        image  = DepthImage(onRealTime: false)
        parameters["Lambda"]    = 0.0
        parameters["Delta"]     = 0.3
        parameters["Epsilon"]   = 0.06
    }
    
    func reallocateVoxels() {
        /* Sequence - serial */
        let count = totalOfVoxels()
        //let square = size * size
        voxels.removeAll()
        voxels.reserveCapacity(totalOfVoxels())
        voxels = [Voxel](repeating: Voxel(), count: count)
        //centroids = [Vector](repeating: Point3D(0, 0, 0), count: count)
        /*
        let stride = MemoryLayout<Point3D>.stride
        let byteCount = stride * count
        let points = UnsafeMutablePointer<Point3D>.allocate(capacity: byteCount)
        bridge_initializeCentroids(points, Int32(numberOfVoxels), resolutionInMeter)
        let buffer = UnsafeBufferPointer(start: points, count: count)
        centroids = Array(buffer)
        points.deallocate()
        */
    }
    
    func reallocateVoxels(with: Int) {
        numberOfVoxels = with
        reallocateVoxels()
    }
    
    func totalOfVoxels() -> Int {
        return numberOfVoxels * numberOfVoxels * numberOfVoxels
    }
    
    func update(intrinsics: matrix_float3x3) {
        camera.update(intrinsics: intrinsics)
    }
    
    func update(extrinsics: matrix_float4x4) {
        camera.update(extrinsics: extrinsics)
    }
    
    func update(extrinsics: matrix_float4x3) {
        camera.update(_extrinsics: extrinsics)
    }
    
    func switchTo(realTime: Bool) {
        camera.changeTo(realTime: realTime)
        image.changeTo(realTime: realTime)
    }
    
    func update(data: [Float]) {
        image.update(_data: data)
    }
    
    func integrate() {
        let width = image.width
        let height = image.height
        var dethmap = image.data
        var Rt = camera.extrinsics
        var K = camera.intrinsics
        let resolve = [resolutionInMeter, resolutionInMeter, resolutionInMeter]
        _ = bridge_integrateDepthMap(&dethmap, /*centroids,*/
            &Rt,
            &K,
            &voxels,
            Int32(width),
            Int32(height),
            Int32(numberOfVoxels),
            resolve,
            parameters["Delta"]!, parameters["Epsilon"]!, parameters["Lambda"]!);
    }
    
    func fullResolution() -> [Float] {
        return [resolutionInMeter, resolutionInMeter, resolutionInMeter]
    }
    /*
    func integrateDepthMap(image: DepthImage, camera: Camera, parameters: [Float]) {
        let width = image.width
        let height = image.height
        var dethmap = image.data
        var Rt = camera.extrinsics
        var K = camera.intrinsics
        let resolve = [resolutionInMeter, resolutionInMeter, resolutionInMeter]
        _ = bridge_integrateDepthMap(&dethmap, /*centroids,*/
            &Rt,
            &K,
            &voxels,
            Int32(width),
            Int32(height),
            Int32(numberOfVoxels),
            resolve,
            parameters[0], parameters[1], parameters[2]);
        //let voxel2 = voxels[numberOfVoxels].sdf
    }
    */
}
