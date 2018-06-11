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
    
    var raytracingEnable: Bool = false
    var cameraPoseEstimationEnable: Bool = false
    
    var voxelResolution: Float
    var dimension:   Int
    var voxels: [Voxel]
    var camera: Camera
    var image: DepthImage
    var parameters: [String: Float] = [String: Float]()
    
    private init() {
        dimension  = 256
        voxelResolution   = 0.02
        voxels = [Voxel](repeating: Voxel(), count: Int(pow(Float(dimension), 3.0)))
        camera = Camera(onRealTime: false)
        image  = DepthImage(onRealTime: false)
        parameters["Lambda"]    = 0.0
        parameters["Delta"]     = 0.02
        parameters["Epsilon"]   = 0.01
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
        dimension = with
        reallocateVoxels()
    }
    
    func totalOfVoxels() -> Int {
        return dimension * dimension * dimension
    }
    
    func update(intrinsics: matrix_float3x3) {
        camera.update(intrinsics: intrinsics)
    }
    
    func update(intrinsics: matrix_float4x4) {
        camera.update(intrinsics: intrinsics)
    }
    
    func update(rotation: matrix_float4x4) {
        let r = getRotationFrom(matrix: rotation)
        camera.update(rotation: r)
    }
    
    func update(translation: matrix_float4x4) {
        let t = getTranslationFrom(matrix: translation)
        camera.update(translation: t)
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
        var dethmap = image.data
        //var Rt = camera.extrinsics
        var R = camera.rotation
        var T = camera.translation
        var K = camera.intrinsics
        if raytracingEnable {
            _ = bridge_raycastDepthMap(
                &dethmap, /*centroids,*/
                &R,
                &T,
                &K,
                &voxels,
                Int32(image.width),
                Int32(image.height),
                Int32(dimension),
                voxelResolution,
                parameters["Delta"]!, parameters["Epsilon"]!, parameters["Lambda"]!)
        }
        else {
            _ = bridge_integrateDepthMap(
                &dethmap, /*centroids,*/
                &R,
                &T,
                &K,
                &voxels,
                Int32(image.width),
                Int32(image.height),
                Int32(dimension),
                voxelResolution,
                parameters["Delta"]!, parameters["Epsilon"]!, parameters["Lambda"]!)
        }
    }
    
    func reinitExtrinsics() {
        voxels = [Voxel](repeating: Voxel(), count: Int(pow(Float(dimension), 3.0)))
        //camera.extrinsics = matrix_float4x4(diagonal: float4(1,1,1,1))
        camera.rotation = matrix_float3x3(diagonal: float3(1,1,1))
        camera.translation = float3(0,0,0)
    }
    
    private func getRotationFrom(matrix: matrix_float4x4) -> matrix_float3x3 {
        let tmp = matrix_float3x4(matrix.columns.0, matrix.columns.1, matrix.columns.2)
        return matrix_float3x3(tmp.transpose.columns.0, tmp.transpose.columns.1, tmp.transpose.columns.2).transpose
    }
    
    private func getTranslationFrom(matrix: matrix_float4x4) -> float3 {
        return float3(matrix.columns.3.x, matrix.columns.3.y, matrix.columns.3.z)
    }
}
