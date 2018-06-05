//
//  io.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 25/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

func importDepthMapFromTXT(from: String, at: String) -> [Float] {
    let relativPath = "/\(at)/\(from)"
    var text: String = ""
    if let path = Bundle.main.path(forResource: relativPath, ofType: "txt") {
        do {
            text = try String(contentsOfFile: path)
        }
        catch { print(error) }
        var rows = text.components(separatedBy: .newlines)
        let a = rows.removeLast()
        if a != "" {
            rows.append(a)
        }
        
        return rows.map {
            $0.components(separatedBy: " ").map { Float($0)! * 1e-3 }
            }.flatMap { $0 }
    }
    return [Float]()
}

func importCameraPose(from: String, at: String) -> matrix_float4x4 {
    let relativPath = "/\(at)/\(from)"
    if let path = Bundle.main.path(forResource: relativPath, ofType: "txt") {
        do {
            let text = try String(contentsOfFile: path)
            let rows = text.components(separatedBy: .newlines)
            let row1 = rows[0].components(separatedBy: .whitespaces)
            let row2 = rows[2].components(separatedBy: .whitespaces)
            let row3 = rows[4].components(separatedBy: .whitespaces)
            let row4 = rows[6].components(separatedBy: .whitespaces)
            return matrix_float4x4(rows: [
                float4(row1.map { Float($0)! } ),
                float4(row2.map { Float($0)! } ),
                float4(row3.map { Float($0)! } ),
                float4(row4.map { Float($0)! } )
                ])
        }
        catch { print(error) }
    }
    return matrix_float4x4()
}

func importCameraIntrinsics(from: String, at: String) -> matrix_float3x3 {
    let relativPath = "/\(at)/\(from)"
    if let path = Bundle.main.path(forResource: relativPath, ofType: "txt") {
        do {
            let text = try String(contentsOfFile: path)
            let rows = text.components(separatedBy: .newlines)
            let row1 = rows[0].components(separatedBy: .whitespaces)
            let row2 = rows[2].components(separatedBy: .whitespaces)
            let row3 = rows[4].components(separatedBy: .whitespaces)
            return matrix_float3x3(rows: [
                float3(Float(row1[0])!, Float(row1[1])!, Float(row1[2])!),
                float3(Float(row2[0])!, Float(row2[1])!, Float(row2[2])!),
                float3(Float(row3[0])!, Float(row3[1])!, Float(row3[2])!)
                ])
        }
        catch { print(error) }
    }
    return matrix_float3x3()
}

func exportToPLY(volume: Model, at: String) {
    let size = volume.numberOfVoxels * volume.numberOfVoxels * volume.numberOfVoxels
    let sdfs = volume.voxels.map { $0.sdf }
    var v = [Vector(0,0,0)]
    if let dir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first {
        let cFileName = dir.appendingPathComponent(at).absoluteString.cString(using: .utf8)
        //bridge_exportVolumeToPLY(volume.centroids, sdfs, cFileName, Int32(size))
        bridge_exportVolumeToPLY(&v, sdfs, cFileName, Int32(size))
    }
}

func exportToPLY(mesh: [Vector], at: String) {
    let numberOfTriangles = mesh.count / 3
    if let dir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first {
        let cFileName = dir.appendingPathComponent(at).absoluteString.cString(using: .utf8)
        bridge_exportMeshToPLY(mesh, cFileName, Int32(numberOfTriangles))
    }
}
