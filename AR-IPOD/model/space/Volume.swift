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
    static let tau_min: Float = 0.4
    
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
        let (voxelsIDs, voxelsPos) = retriveIDs(from: bbox, dim: size, step: resolution)
        // For each voxel/centroid retrieved
        for i in 0..<voxelsIDs.count {
            let id = voxelsIDs[i]
            let pos = voxelsPos[i]
            // Check if centroids exists
            if centroids.index(forKey: id) == nil {
                centroids[id] = pos
            }
            // Check if voxels exists
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
                    voxels[id]?.carve()
                }
                else if fabs(distance) >= tau + range {
                    voxels[id]?.update(sdfUpdate: uv.x, weightUpdate: 1.0)
                }
            }
        }
    }
}
