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
    var type: CameraType = .Other
    
    var voxelResolution: Float
    var dimension:   Int
    var voxels: [Voxel]
    var camera: Camera
    var image: DepthImage
    var parameters: [String: Float] = [String: Float]()
    
    private init() {
        dimension       = 256
        voxelResolution = 0.02
        voxels          = [Voxel](repeating: Voxel(), count: Int(pow(Float(dimension), 3.0)))
        camera  = KinectCamera()
        image   = KinectDepthImage()
        type    = .Kinect
        parameters["Lambda"]    = 0.0
        parameters["Delta"]     = 0.02
        parameters["Epsilon"]   = 0.01
    }
    
    private init(type: CameraType) {
        dimension       = 256
        voxelResolution = 0.02
        voxels          = [Voxel](repeating: Voxel(), count: Int(pow(Float(dimension), 3.0)))
        switch type {
        case .Kinect:
            camera  = KinectCamera()
            image   = KinectDepthImage()
        case .Iphone:
            camera  = IphoneCamera()
            image   = IphoneDepthImage()
        default:
            camera  = Camera()
            image   = DepthImage()
        }
        self.type               = type
        parameters["Lambda"]    = 0.0
        parameters["Delta"]     = 0.02
        parameters["Epsilon"]   = 0.01
    }
    
    init(from: Model, to: CameraType) {
        switch to {
        case .Kinect:
            camera  = KinectCamera()
            image   = KinectDepthImage()
        case .Iphone:
            camera  = IphoneCamera()
            image   = IphoneDepthImage()
        default:
            camera  = Camera()
            image   = DepthImage()
        }
        type            = to
        dimension       = from.dimension
        voxelResolution = from.voxelResolution
        voxels          = from.voxels
        parameters      = from.parameters
    }
    
    func reallocateVoxels() {
        /* Sequence - serial */
        let count = numberOfVoxels()
        //let square = size * size
        voxels.removeAll()
        voxels.reserveCapacity(numberOfVoxels())
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
    
    func reallocateVoxels(amount: Int) {
        dimension = amount
        reallocateVoxels()
    }
    
    func numberOfVoxels() -> Int {
        return dimension * dimension * dimension
    }
    
    func update(intrinsics: matrix_float3x3) {
        var tmp = matrix_float4x4()
        tmp.columns.0 = float4(intrinsics.columns.0.x, intrinsics.columns.0.y, intrinsics.columns.0.z, 0);
        tmp.columns.1 = float4(intrinsics.columns.1.x, intrinsics.columns.1.y, intrinsics.columns.1.z, 0);
        tmp.columns.2 = float4(intrinsics.columns.2.x, intrinsics.columns.2.y, intrinsics.columns.2.z, 0);
        tmp.columns.3 = float4(0, 0, 0, 1);
        camera.update(intrinsics: tmp)
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
    
    func switchTo(type: CameraType) {
        self.type    = type
        camera  = camera.switchTo(type: type)
        image   = image.switchTo(type: type)
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
 
    func reinit() {
        reallocateVoxels()
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
