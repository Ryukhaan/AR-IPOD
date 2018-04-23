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
    static let tau_min: Float = 0.5
    
    var size:       Point3D // X size, Y size and Z size
    var resolution: Float   // Number of voxels per meter
    var voxels:     [Int: Voxel] = [Int: Voxel]()
    var centroids:  [Int: Vector] = [Int: Vector]()
    
    init(_size: Point3D, _resolution: Float) {
        size        = _size
        resolution  = _resolution
        allocate()
        
    }
    
    private mutating func allocate() {
        voxels.removeAll()
        voxels.reserveCapacity(numberOfVoxels())
        centroids.removeAll()
        centroids.reserveCapacity(numberOfVoxels())
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
        let voxelsIDs = retriveIDs(from: bbox, dim: size, step: resolution)
        // For each voxel/centroid retrieved
        for id in voxelsIDs {
            if centroids.index(forKey: id) == nil {
                var pos = Point3D.inverse(n: id)
                let round = 1.0 / resolution
                pos.x = (round * pos.x - 0.5) * size.x
                pos.y = (round * pos.y - 0.5) * size.y
                pos.z = (round * pos.z - 0.5) * size.z
                centroids[id] = pos
            }
            if voxels.index(forKey: id) == nil {
                voxels[id] = Voxel()
            }
            
            if let centroid = centroids[id] {
                let positionCamera = camera.extrinsics.columns.3
                let distance = (centroid - Vector(positionCamera.x, positionCamera.y, positionCamera.z)).length()
                let uv      = camera.project(vector: centroid)
                let depth   = image.at(row: Int(uv.x), column: Int(uv.y))
                if depth.isNaN { continue }
                let proj    = camera.unproject(pixel: uv, depth: Float(depth))
                let range   = proj.length()
                let tau     = truncation(range: range)
                if distance >= Volume.tau_min && distance < tau - range {
                    voxels[id]?.update(sdfUpdate: tau, weightUpdate: 0.5)
                }
                else if distance >= tau - range && distance <= tau + range {
                    voxels[id]?.update(sdfUpdate: uv.x, weightUpdate: 1.0)
                }
            }
        }
    }
}
