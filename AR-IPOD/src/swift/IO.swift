//
//  io.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 25/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

func importPointCloud(fromFile: String) -> PointCloud {
    var text: String = ""
    let pc = PointCloud()
    if let path = Bundle.main.path(forResource: "pointCloudTest", ofType: "sdp") {
        do {
            try text = String(contentsOfFile: path, encoding: .utf8)
        }
        catch {
            print("Sorry could not load file pointCloudTest.sdp !")
        }
    }
    else {
        print("Point cloud file not found !")
    }
    let rows = text.components(separatedBy: .newlines)
    for row in rows {
        let temp = row.components(separatedBy: " ")
        if temp.count < 1 { continue }
        pc.addPoint(point: Vector(Float(temp[0])!, Float(temp[1])!, Float(temp[2])!))
    }
    return pc
}

func importDepthMap(fromFile: String) -> [Float] {
    var text: String = ""
    var outArray = [Float]()
    do {
        try text = String(contentsOfFile: fromFile, encoding: .utf8)
    }
    catch {}
    let rows = text.components(separatedBy: .newlines)
    for row in rows {
        let temp = row.components(separatedBy: " ")
        if temp.count < 1 { print(temp.count); continue }
        outArray.append(Float(temp[2])! + 0.5)
    }
    return outArray
}

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

func exportToPLY(volume: Volume, at: String) {
    let size = volume.numberOfVoxels * volume.numberOfVoxels * volume.numberOfVoxels
    let sdfs = volume.voxels.map { $0.sdf }
    if let dir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first {
        let cFileName = dir.appendingPathComponent(at).absoluteString.cString(using: .utf8)
        bridge_exportVolumeToPLY(volume.centroids, sdfs, cFileName, Int32(size))
    }
}

func exportToPLY(mesh: [Vector], at: String) {
    let numberOfTriangles = mesh.count / 3
    if let dir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first {
        let cFileName = dir.appendingPathComponent(at).absoluteString.cString(using: .utf8)
        bridge_exportMeshToPLY(mesh, cFileName, Int32(numberOfTriangles))
    }
}

func exportPointCloud(points: [Vector], at: String) {
    var text: String = ""
    if let dir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first {
        let cFileName = dir.appendingPathComponent(at)
        for i in 0..<points.count {
            let point = points[i]
            text = "\(text)\(point.x) \(point.y) \(point.z)\n"
        }
        do {
            try text.write(to: cFileName, atomically: false, encoding: .utf8)
        }
        catch {}
    }
}
