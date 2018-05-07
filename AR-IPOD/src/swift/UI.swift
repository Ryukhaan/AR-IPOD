//
//  UI.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 07/05/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

func createSimpleNode(points: [Vector]) -> SCNNode {
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
