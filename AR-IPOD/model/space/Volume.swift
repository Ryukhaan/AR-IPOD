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
    var id:         Point3D
    let resolution: Float
    var origin:     Vector
    let dimensions: Point3D
    var voxels:     [Voxel] = [Voxel]()
    
    init(_id: Point3D, _dimensions: Point3D, _resolution: Float) {
        id          = _id
        dimensions  = _dimensions
        resolution  = _resolution
        origin      = Vector(Float(dimensions.x * id.x) * resolution,
                             Float(dimensions.y * id.y) * resolution,
                             Float(dimensions.z * id.z) * resolution)
        allocate()
    }
    
    mutating func allocate() {
        voxels.removeAll()
        voxels.reserveCapacity(self.size())
    }
    
    func getVoxelID(i: Int, j: Int, k: Int) -> Int {
        return (k * Int(dimensions.y) + j) * Int(dimensions.x) + i
    }
    
    func computeBoundingBox() -> Box {
        let size = resolution * Vector(Float(dimensions.x), Float(dimensions.y), Float(dimensions.z))
        return Box(min: origin, max: origin + size)
    }
    
    func size() -> Int {
        return Int(dimensions.x * dimensions.y * dimensions.z)
    }
    
    func voxelAt(i: Int, j: Int, k: Int) -> Voxel {
        return voxels[getVoxelID(i: i, j: j, k: k)]
    }
    
    func voxelAt(coordinate: Point3D) -> Voxel {
        return voxels[getVoxelID(i: Int(coordinate.x), j: Int(coordinate.y), k: Int(coordinate.z))]
    }
    
    func voxelAtFromWorld(worldPoint: Point3D) -> Voxel {
        return voxelAt(coordinate: localCoordinate(worldPoint: worldPoint))
    }
    
    func voxelCoordinate(worldVector: Vector) -> Point3D {
        let roundX = 1.0 / (resolution * Float(dimensions.x))
        let roundY = 1.0 / (resolution * Float(dimensions.y))
        let roundZ = 1.0 / (resolution * Float(dimensions.z))
        return Point3D(floor(worldVector.x) * roundX,
                       floor(worldVector.y) * roundY,
                       floor(worldVector.z) * roundZ)
    }
    
    func voxelAtFromWorld(worldVector: Vector) -> Voxel {
        return voxelAt(coordinate: voxelCoordinate(worldVector: worldVector))
    }
    
    func isEmpty() -> Bool {
        return voxels.isEmpty
    }
    
    func localCoordinate(worldPoint: Point3D) -> Point3D {
        return worldPoint - Point3D(id.x * dimensions.x,
                                     id.y * dimensions.y,
                                     id.z * dimensions.z)
    }
    
    func index() -> Int {
        let g = { (x: Int, y: Int) -> Int in
            let x1 = x + y
            return x1 * (x1 + 1) / 2 + y
            
        }
        return g(g(Int(id.x), Int(id.y)), Int(id.z))
    }
    
}
