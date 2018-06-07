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
    
    var dimension: Float
    var resolution:   Int
    var voxels: [Voxel]
    var camera: Camera
    var image: DepthImage
    var parameters: [String: Float] = [String: Float]()
    
    private init() {
        resolution  = 256
        dimension   = 8.0
        voxels = [Voxel](repeating: Voxel(), count: Int(pow(Float(resolution), 3.0)))
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
        resolution = with
        reallocateVoxels()
    }
    
    func totalOfVoxels() -> Int {
        return resolution * resolution * resolution
    }
    
    func update(intrinsics: matrix_float3x3) {
        camera.update(intrinsics: intrinsics)
    }
    
    func update(intrinsics: matrix_float4x4) {
        camera.update(intrinsics: intrinsics)
    }
    
    func update(extrinsics: matrix_float4x4, onlyRotation: Bool) {
        camera.update(extrinsics: extrinsics, onlyRotation: onlyRotation)
    }
    
    func switchTo(realTime: Bool) {
        camera.changeTo(realTime: realTime)
        image.changeTo(realTime: realTime)
    }
    
    func update(data: [Float]) {
        image.update(_data: data)
    }
    
    func update(data: UnsafeMutablePointer<Float>) {
        image.update(_data: data)
    }
    
    func createMedianDepthMap() {
        image.updateDataWithSavedData()
    }
    
    func integrate() {
        let width = image.width
        let height = image.height
        var dethmap = image.data
        var Rt = camera.extrinsics
        var K = camera.intrinsics
        let resolve = [dimension, dimension, dimension]
        //if let dir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first {
        //    let cFileName = dir.appendingPathComponent("points.sdp").absoluteString.cString(using: .utf8)
        //_ = bridge_raycastDepthMap(
        _ = bridge_integrateDepthMap(
            &dethmap, /*centroids,*/
            &Rt,
            &K,
            &voxels,
            Int32(width),
            Int32(height),
            Int32(resolution),
            resolve,
            parameters["Delta"]!, parameters["Epsilon"]!, parameters["Lambda"]!)
        //cFileName)
        //}
    }
    
    func getDimensions() -> [Float] {
        return [dimension, dimension, dimension]
    }
    
    func reinit() {
        voxels = [Voxel](repeating: Voxel(), count: Int(pow(Float(resolution), 3.0)))
        camera = Camera(onRealTime: false)
        image  = DepthImage(onRealTime: false)
    }
}
