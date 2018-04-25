//
//  io.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 25/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

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

func exportToPlyFormat(volume: Volume, fileName: String) {
    var text: String = ""
    let header = "ply \n"
    let formatHeader = "format ascii 1.0 \n"
    let vertexCount = "element vertex \(volume.centroids.count) \n"
    let propertyX   = "property float x \n"
    let propertyY   = "property float y \n"
    let propertyZ   = "property float z \n"
    let red         = "property uchar red \n"
    let green       = "property uchar green \n"
    let blue        = "property uchar blue \n"
    //let alpha        = "property uchar alpha \n"
    //let faceCount = "element face \(volume.centroids.count * 6) \n"
    //let faceListProperty = "property list uchar int vertex_index \n"
    let endHeader = "end_header \n"
    
    print("Start Writing ! \n")
    if let dir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first {
        let fileURL = dir.appendingPathComponent(fileName)
        print(fileURL)
        // Writing Header
        text += "\(header)\(formatHeader)\(vertexCount)\(propertyX)\(propertyY)\(propertyZ)\(red)\(blue)\(green)\(endHeader)"

        // Writing vertex
        let number = volume.centroids.count / 4
        for i in 0..<number {
            let c1 = volume.centroids[4*i]
            let c2 = volume.centroids[4*i+1]
            let c3 = volume.centroids[4*i+2]
            let c4 = volume.centroids[4*i+3]
            let v1 = 255-min(Int(volume.voxels[4*i].sdf), 255)
            let v2 = 255-min(Int(volume.voxels[4*i].sdf), 255)
            let v3 = 255-min(Int(volume.voxels[4*i].sdf), 255)
            let v4 = 255-min(Int(volume.voxels[4*i].sdf), 255)
            text += "\(c1.x) \(c1.y) \(c1.z) \(v1) 54 13 \n \(c2.x) \(c2.y) \(c2.z) \(v2) 54 13 \n \(c3.x) \(c3.y) \(c3.z) \(v3) 54 13\n \(c4.x) \(c4.y) \(c4.z) \(v4) 54 13\n"
        }
        do {
            try text.write(to: fileURL, atomically: false, encoding: .utf8)
        }
        catch {}
    }
}
