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

func importCameraIntrinsics(from: String, at: String) -> matrix_float4x4 {
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
                float4(Float(row1[0])!, Float(row1[1])!, Float(row1[2])!, Float(row1[3])!),
                float4(Float(row2[0])!, Float(row2[1])!, Float(row2[2])!, Float(row2[3])!),
                float4(Float(row3[0])!, Float(row3[1])!, Float(row3[2])!, Float(row3[3])!),
                float4(Float(row4[0])!, Float(row4[1])!, Float(row4[2])!, Float(row4[3])!)
                ])
        }
        catch { print(error) }
    }
    return matrix_float4x4()
}

func exportToPLY(volume: Model, at: String) {
    let size = volume.dimension * volume.dimension * volume.dimension
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

func save(model: Model, atTime: Int) {
    let intrinsicsPath  = "depthIntrinsics.txt"
    let extrinsicsPath  = "frame-\(atTime).pose.txt"
    let depthmapPath    = "frame-\(atTime).depth.txt"
    let K   = model.camera.intrinsics
    let Rt  = model.camera.extrinsics
    let Dm  = model.image.data
    let size = Int(model.image.width * model.image.height)
    if let dir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first {
        let KPath   = dir.appendingPathComponent(intrinsicsPath)
        let RtPath  = dir.appendingPathComponent(extrinsicsPath)
        let DmPath  = dir.appendingPathComponent(depthmapPath)
        // Write K
        do {
            try "\(K.columns.0.x) \(K.columns.1.x) \(K.columns.2.x) 0\n".write(to: KPath, atomically: false, encoding: .utf8)
            try "\(K.columns.0.y) \(K.columns.1.y) \(K.columns.2.y) 0\n".write(to: KPath, atomically: false, encoding: .utf8)
            try "\(K.columns.0.z) \(K.columns.1.z) \(K.columns.2.z) 0\n".write(to: KPath, atomically: false, encoding: .utf8)
            try "0 0 0 1".write(to: KPath, atomically: false, encoding: .utf8)
        }
        catch {}
        // Write Rt
        do {
            try "\(Rt.columns.0.x) \(Rt.columns.1.x) \(Rt.columns.2.x) \(Rt.columns.3.x)\n".write(to: RtPath, atomically: false, encoding: .utf8)
            try "\(Rt.columns.0.y) \(Rt.columns.1.y) \(Rt.columns.2.y) \(Rt.columns.3.y)\n".write(to: RtPath, atomically: false, encoding: .utf8)
            try "\(Rt.columns.0.z) \(Rt.columns.1.z) \(Rt.columns.2.z) \(Rt.columns.3.z)\n".write(to: RtPath, atomically: false, encoding: .utf8)
            try "0 0 0 1".write(to: KPath, atomically: false, encoding: .utf8)
        }
        catch {}
        // Write DepthMap
        do {
            for i in 0..<size {
                let depth = Dm[i]
                if (i+1) % Int(model.image.width) == 0 {
                    try "\(depth) \n".write(to: DmPath, atomically: false, encoding: .utf8)
                }
                else {
                    try "\(depth)".write(to: DmPath, atomically: false, encoding: .utf8)
                }
            }
        }
        catch {}
    }
}
