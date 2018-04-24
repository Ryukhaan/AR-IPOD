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

struct Volume {
    // Singleton pattern : Only one volume will be create
    static let sharedInstance = Volume()
    
    static let tau_min: Float = 0.4
    
    var resolution: Float   // Number of voxels per meter
    var size:       Point3D // X size, Y size and Z size
    // Since i now the hash function, is adictionary still necessary ?
    //lazy var voxels:     [Int: Voxel]   = self.allocate()
    //lazy var centroids:  [Int: Vector]  = self.allocate()
    lazy var voxels:    [Voxel]     = self.allocate()
    lazy var centroids: [Vector]    = self.allocate()
    
    // Prevents others from using default init() for this class
    private init() {
    //init() {
        size        = Point3D(256, 256, 256)
        resolution  = 1.0
    }
    
    func allocate<T>() -> [T] {
        var allocator = [T]()
        allocator.removeAll()
        allocator.reserveCapacity(numberOfVoxels())
        return allocator
    }
    
    mutating func initialize() {
        let count = Int(size.x * size.y * size.z)
        voxels = [Voxel](repeating: Voxel(), count: count)
        centroids = [Vector](repeating: Point3D(0, 0, 0), count: count)
        centroids = (0..<count).map {
                let x = Float($0 / 65536)
                let remainder = $0 % (65536)
                let y = Float(remainder / 256)
                let z = Float(remainder % 256)
                return Point3D(x, y, z)
        }
        
        centroids = centroids.map {
            mappingVoxelToCentroid(voxel: $0,
                                   dim: Int(size.x),
                                   voxelResolution: resolution)
        }
    }
    
    func numberOfVoxels() -> Int {
        return Int(size.x * size.y * size.z)
    }
    
    func truncation(range: Float) -> Float {
        // TO DO
        return 1
    }
    
    mutating func integrateDepthMap(image: DepthImage, camera: Camera) {
        // Get nearest and farthest depth
        let (minimumDepth, maximumDepth, _) = image.getStats()
        // Set near range and far range
        var copyCamera = camera
        copyCamera.zFar = maximumDepth
        copyCamera.zNear = minimumDepth
        // Create camera frustum
        let frustrum = Frustrum()
        frustrum.setUp(camera: copyCamera)
        // Determines intersects between frustrum and volume
        let bbox = computeBoundingBox(frustrum: frustrum)
        let (voxelsIDs, _) = retriveIDs(from: bbox, dim: size, step: resolution)
        // For each voxel/centroid retrieved
        for i in 0..<voxelsIDs.count {
            let id = voxelsIDs[i]
            let centroid = centroids[id]
            let positionCamera = camera.extrinsics.columns.3
            let distance = (centroid - Vector(positionCamera.x, positionCamera.y, positionCamera.z)).length()
            let uv      = camera.project(vector: centroid)
            let depth   = image.at(row: Int(uv.x), column: Int(uv.y))
            if depth.isNaN { continue }
            let proj    = camera.unproject(pixel: uv, depth: Float(depth))
            let range   = proj.length()
            let tau     = truncation(range: range)
            if distance >= Volume.tau_min && distance < tau - range {
                voxels[id].carve()
            }
            else if fabs(distance) >= tau + range {
                voxels[id].update(sdfUpdate: uv.x, weightUpdate: 1)
            }
        }
    }
}
