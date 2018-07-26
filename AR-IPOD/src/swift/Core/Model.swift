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
        voxelResolution = 0.006
        voxels          = [Voxel](repeating: Voxel(), count: Int(pow(Float(dimension), 3.0)))
        camera  = IphoneCamera()
        image   = IphoneDepthImage()
        type    = .iPhoneX
        parameters["Lambda"]    = 0.0
        parameters["Delta"]     = 0.3
        parameters["Epsilon"]   = 0.025
        parameters["cx"]        = 6
        parameters["cy"]        = 6
    }
    
    private init(type: CameraType) {
        dimension       = 256
        voxelResolution = 0.01
        voxels          = [Voxel](repeating: Voxel(), count: Int(pow(Float(dimension), 3.0)))
        switch type {
        case .Kinect:
            camera  = KinectCamera()
            image   = KinectDepthImage()
        case .iPhoneX:
            camera  = IphoneCamera()
            image   = IphoneDepthImage()
        default:
            camera  = Camera()
            image   = DepthImage()
        }
        self.type               = type
        parameters["Delta"]         = 0.3
        parameters["Epsilon"]       = 0.025
        parameters["icpMaxIter"]    = 1
        parameters["icpMaxDist"]    = 2.0
        parameters["icpMaxCorr"]    = 0.2
    }
    
    init(from: Model, to: CameraType) {
        switch to {
        case .Kinect:
            camera  = KinectCamera()
            image   = KinectDepthImage()
        case .iPhoneX:
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
        let count = numberOfVoxels()
        voxels.removeAll()
        voxels.reserveCapacity(numberOfVoxels())
        voxels = [Voxel](repeating: Voxel(), count: count)
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
        tmp.columns.0 = float4(intrinsics.columns.0, 0);
        tmp.columns.1 = float4(intrinsics.columns.1, 0);
        tmp.columns.2 = float4(intrinsics.columns.2, 0);
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
    
    func push(data: UnsafeMutablePointer<Float>) {
        image.push(map: data)
    }
    
    func computeDepthsMedian() {
        image.collect()
    }
    
    func integrate() {
        var dethmap = image.data
        //var Rt = camera.extrinsics
        var R = camera.rotation
        var T = camera.translation
        var K = camera.intrinsics
        bridge_integrateDepthMap(
            &dethmap, /*centroids,*/
            &R,
            &T,
            &K,
            &voxels,
            Int32(image.width),
            Int32(image.height),
            Int32(dimension),
            voxelResolution,
            parameters["Delta"]!, parameters["Epsilon"]!, parameters["Lambda"]!,
            parameters["cx"]!, parameters["cy"]!)
    }
    
    func reinit() {
        reallocateVoxels()
        camera.rotation = matrix_float3x3(diagonal: float3(1,1,1))
        camera.translation = float3(0,0,0)
    }
    
    private func getRotationFrom(matrix: matrix_float4x4) -> matrix_float3x3 {
        return matrix_float3x3(
            float3(matrix.columns.0),
            float3(matrix.columns.1),
            float3(matrix.columns.2))
    }
    
    private func getTranslationFrom(matrix: matrix_float4x4) -> float3 {
        return float3(matrix.columns.3.x, matrix.columns.3.y, matrix.columns.3.z)
    }
    
    func toGlobal(point: simd_int3, resolution: Float) -> float3 {
        return resolution * Vector(point)
    }
    
    func integrateDepth() {
        let R = camera.rotation
        let T = camera.translation
        let K = camera.intrinsics
        let Kinv = simd_inverse(K)
        let width = image.width
        let height = image.height
        guard let cx = parameters["cx"] else { return }
        guard let cy = parameters["cy"] else { return }
        guard let delta = parameters["Delta"] else { return }
        guard let epsilon = parameters["Epsilon"] else { return }
        var n = -1
        let globalOffset = 0.5 * Float(dimension) * voxelResolution
        for i in 0..<dimension {
            for j in 0..<dimension {
                for k in 0..<dimension {
                    n += 1
                    //if (n <= mini || n >= maxi) continue;
                    // Retrieve Voxel and Global Vector associated to
                    let v_ijk = simd_make_int3(Int32(i), Int32(j), Int32(k))
                    let centroid = toGlobal(point: v_ijk, resolution: voxelResolution) - globalOffset
                    
                    // Back to Local World
                    let X_L      = simd_mul(simd_transpose(R), centroid - T)
                    if (X_L.z < 0) { continue }
                    
                    // Project to Screen World
                    let homogeneous = float4(X_L)
                    let project  = simd_mul(K, homogeneous)
                    //print(project)
                    let u = Int(project.x / (project.z  * cx))
                    let v = Int(project.y / (project.z  * cy))
                    if (u < 0 || u >= height) { continue }
                    if (v < 0 || v >= width) { continue }
                    
                    // Get depth at pixel (u, v)
                    let zp = image.data[u * width + v];
                    if (zp.isNaN) { continue }
                    if (zp < 1e-7) { continue }
                    
                    // Compute distance
                    let uvz = simd_make_float4(zp * Float(u) * cx, zp * Float(v) * cy, zp, 1.0)
                    let X_S = simd_mul(Kinv, uvz)
                    let distance = simd_sign(zp - X_L.z) * simd_distance(X_S , homogeneous);
                    let weight: Float = 1.0
                    
                    // Update Voxel at index n
                    let updated_voxel = voxels[n]
                    if (distance >= delta + epsilon && distance < zp) {
                        voxels[n] = updated_voxel.carving()
                    }
                    if (fabs(distance) <= delta) {
                        voxels[n] = updated_voxel.update(sdf: distance, weight: weight)
                    }
                }
            }
        }
    }
}
