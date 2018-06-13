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
    static func createPointsNode(points: [Vector]) -> SCNNode {
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
    
    static func createMeshNode(points: [Vector]) -> SCNNode {
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
            primitiveType: .triangles,
            primitiveCount: points.count / 3,
            bytesPerIndex: MemoryLayout<Double>.size
        )
        let pointCloudGeometry = SCNGeometry(sources: [positionSources], elements: [elements])
        return SCNNode(geometry: pointCloudGeometry)
    }
    
    static func createSimpleNodeFrom(volume: Model, with: Float) -> SCNNode {
        let points = [Vector]()
        let size = volume.numberOfVoxels()
        for i in 0..<size {
            if (abs(volume.voxels[i].sdf) <= with) {
                //points.append(from.centroids[i])
            }
        }
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
            bytesPerIndex: MemoryLayout<Double>.size
        )
        let pointCloudGeometry = SCNGeometry(sources: [positionSources], elements: [elements])
        return SCNNode(geometry: pointCloudGeometry)
    }
}
