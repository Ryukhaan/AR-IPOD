//
//  UI.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 07/05/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

enum UIFactory {
    
    static func pointCloud(points: [Vector]) -> SCNNode {
        let vertexData = NSData(bytes: points, length: MemoryLayout<Vector>.stride * points.count)
        let positionSources = SCNGeometrySource(data: vertexData as Data,
                                                semantic: SCNGeometrySource.Semantic.vertex,
                                                vectorCount: points.count,
                                                usesFloatComponents: true,
                                                componentsPerVector: 3,
                                                bytesPerComponent: MemoryLayout<Float>.size,
                                                dataOffset: 0,
                                                dataStride: MemoryLayout<Vector>.stride)
        let elements = SCNGeometryElement(
            data: nil,
            primitiveType: .point,
            primitiveCount: points.count,
            bytesPerIndex: MemoryLayout<Float>.size
        )
        let pointCloudGeometry = SCNGeometry(sources: [positionSources], elements: [elements])
        return SCNNode(geometry: pointCloudGeometry)
    }
    
    static func mesh(from: [Vector]) -> SCNNode {
        var normals = [SCNVector3]()
        let n = from.count / 3
        for i in 0..<n {
            let p0 = from[3*i]
            let p1 = from[3*i+1]
            let p2 = from[3*i+2]
            let px = (p1 - p0)
            let py = (p2 - p0)
            let n = simd_normalize(simd_cross(px, py))
            normals.append(SCNVector3(n.x, n.y, n.z))
            normals.append(SCNVector3(n.x, n.y, n.z))
            normals.append(SCNVector3(n.x, n.y, n.z))
        }
        let vertexData = NSData(bytes: from, length: MemoryLayout<Vector>.stride * from.count)
        let positionSources = SCNGeometrySource(data: vertexData as Data,
                                                semantic: SCNGeometrySource.Semantic.vertex,
                                                vectorCount: from.count,
                                                usesFloatComponents: true,
                                                componentsPerVector: 3,
                                                bytesPerComponent: MemoryLayout<Float>.size,
                                                dataOffset: 0,
                                                dataStride: MemoryLayout<Vector>.stride)
        let normalSources = SCNGeometrySource(normals: normals)
       // let normalData = NSData(bytes: normals, length: MemoryLayout<Vector>.stride * normals.count)
       /* let normalsSources = SCNGeometrySource(data: normalData as Data,
                                               semantic: SCNGeometrySource.Semantic.normal,
                                               vectorCount: normals.count,
                                               usesFloatComponents: true,
                                               componentsPerVector: 3,
                                               bytesPerComponent: MemoryLayout<Float>.size,
                                               dataOffset: 0,
                                               dataStride: MemoryLayout<Vector>.stride)
        */
        let elements = SCNGeometryElement(
            data: nil,
            primitiveType: .triangles,
            primitiveCount: from.count / 3,
            bytesPerIndex: MemoryLayout<Double>.size
        )
        let pointCloudGeometry = SCNGeometry(sources: [positionSources, normalSources], elements: [elements])
        return SCNNode(geometry: pointCloudGeometry)
    }
}
