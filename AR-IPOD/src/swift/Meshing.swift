//
//  Meshing.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 26/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

/*
 * For C files -> Int32
 * Otherwise -> Int
 */

/**
 * Tables contains the two tables required for marching cubes algorithm.
 * Those tables are saved under "EdgeTableContents" and "TriTableContents", see resources/ directory.
 */
struct Tables {
    static var edgeTable: [Int32]  = {
        var text: String = ""
        if let path = Bundle.main.path(forAuxiliaryExecutable: "EdgeTableContents") {
            do {
                try text = String(contentsOfFile: path, encoding: .utf8)
            }
            catch {
                print("Sorry could not load file EdgeTableContents.rtf !")
            }
        }
        let preprocess = text.components(separatedBy: .whitespacesAndNewlines).joined()
            //.components(separatedBy: " ").joined() // Remove spaces
            .components(separatedBy: "0x").joined() // Remove 0x
        return preprocess.components(separatedBy: ",").map { Int32($0, radix: 16)! }
    }()
    
    static var triTable: [[Int32]] = {
        var text: String = ""
        if let path = Bundle.main.path(forAuxiliaryExecutable: "TriTableContents") {
            do {
                try text = String(contentsOfFile: path, encoding: .utf8)
            }
            catch {
                print("Sorry could not load file TriTableContents.rtf !")
            }
        }
        return text.components(separatedBy: .newlines).map {
            $0.components(separatedBy: " ").joined().components(separatedBy: ",").map { Int32($0)! }
        }
    }()
}

/**
 * Function bridging Marching Cubes Algorithm.
 * @volume : Volume to be "marched"
 * @isolovel : isovalue according to Lorensen & Clide paper (1987)
 */
func extractMesh(model: inout Model, isolevel: Float) -> [Vector] {
    let count = model.numberOfVoxels()
    let stride = MemoryLayout<Vector>.stride
    // Why i can't allocate more than around "count" bytes ?
    let byteCount = 3 * stride * model.dimension * model.dimension
    //let byteCount = 3 * stride
    let triangles = UnsafeMutablePointer<Vector>.allocate(capacity: byteCount)
    defer {
        triangles.deallocate()
    }
    //var sdfs = volume.voxels.map { $0.sdf }
    var tempTri = Tables.triTable.flatMap { $0 }
    let numberOfTriangles = bridge_extractMesh(
        triangles,
        &(model.voxels),
        //&sdfs,
        //&volume.centroids,
        &Tables.edgeTable,
        &tempTri,
        Int32(model.dimension),
        isolevel,
        model.voxelResolution)
    let buffer = UnsafeBufferPointer(start: triangles, count: Int(numberOfTriangles))
    return Array(buffer)
}

func extractTSDF(model: Model, isolevel: Float) -> [Vector] {
    let resolution = model.voxelResolution
    let dim = model.dimension
    let offset = resolution * Float(dim) * 0.5
    let square = model.dimension * model.dimension
    var points = [Vector]()
    for (i, voxel) in model.voxels.enumerated() {
        if voxel.sdf <= isolevel {
            var centroid = Vector(0,0,0)
            let x = Float(i / square)
            let remainder = i % square
            let y = Float(remainder / dim)
            let z = Float(remainder % dim)
            centroid.x = (resolution * (x + 0.5)) - offset
            centroid.y = (resolution * (y + 0.5)) - offset
            centroid.z = (resolution * (z + 0.5)) - offset
            points.append(centroid)
        }
    }
    return points
}

func projectDepthMap(from: Model) -> [Vector] {
    let height = Int(from.camera.height)
    let width = Int(from.camera.width)
    let Kinv = simd_inverse(from.camera.intrinsics)
    var points = [Vector]()
    for i in 0..<height {
        for j in 0..<width{
            let z = from.image.data[i * width + j]
            let uvz = vector4(z * Float(i), z * Float(j), z, 1.0)
            let local = simd_mul(Kinv, uvz)
            points.append(Vector(local.x, local.y, local.z))
        }
    }
    return points
}
