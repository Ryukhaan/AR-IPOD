//
//  Meshing.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 26/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

func extractMesh(volume: Volume, isolevel: Float) -> [Vector] {
    let count = volume.numberOfVoxels()
    let stride = MemoryLayout<Vector>.stride
    // Why i can't allocate more than around "count" bytes ?
    let byteCount = stride * count
    let triangles = UnsafeMutablePointer<Vector>.allocate(capacity: byteCount)
    defer {
        triangles.deallocate()
    }
    var sdfs = volume.voxels.map { $0.sdf }
    var tempTri = Tables.triTable.flatMap { $0 }
    let numberOfTriangles = bridge_extractMesh( triangles,
                       &sdfs,
                       &volume.centroids,
                       &Tables.edgeTable,
                       &tempTri,
                       Int32(volume.size),
                       isolevel)
    let buffer = UnsafeBufferPointer(start: triangles, count: Int(numberOfTriangles))
    return Array(buffer)
}

