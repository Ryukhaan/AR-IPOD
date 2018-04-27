//
//  MarchingCube.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 26/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

/*
func interpolateVertex(isolevel: Float, a: Vector, b: Vector, alpha: Float, beta: Float) -> Vector {
    if (abs(isolevel-alpha) < 0.00001)  { return a }
    if (abs(isolevel-beta) < 0.00001)   { return b }
    if (abs(alpha-beta) < 0.00001)      { return a }
    let mu = (isolevel - alpha) / (beta - alpha)
    return a + mu * (b - a)
}

func polygonise(gridCell: Cell, isolevel: Float) ->  (Int, [Triangle]) {
    var triangles = [Triangle]()
    var ntriangle: Int = 0
    var cubeindex: Int = 0
    var vertexlist = Array<Vector>(repeating: Vector(0,0,0), count: 12)
    
    if gridCell.values[0] < isolevel { cubeindex |= 1 }
    if gridCell.values[1] < isolevel { cubeindex |= 2 }
    if gridCell.values[2] < isolevel { cubeindex |= 4 }
    if gridCell.values[3] < isolevel { cubeindex |= 8 }
    if gridCell.values[4] < isolevel { cubeindex |= 16 }
    if gridCell.values[5] < isolevel { cubeindex |= 32 }
    if gridCell.values[6] < isolevel { cubeindex |= 64 }
    if gridCell.values[7] < isolevel { cubeindex |= 128 }
    /* Cube is entirely in/out of the surface */
    if Tables.edgeTable[cubeindex] == 0 { return (ntriangle, triangles) }
    
    /* Find the vertices where the surface intersects the cube */
    if (Tables.edgeTable[cubeindex] & 1) > 0 {
        vertexlist[0] = interpolateVertex(isolevel: isolevel,a: gridCell.points[0],b: gridCell.points[1],alpha: gridCell.values[0],beta: gridCell.values[1])
    }
    if (Tables.edgeTable[cubeindex] & 2) > 0 {
        vertexlist[1] = interpolateVertex(isolevel: isolevel,a: gridCell.points[1],b: gridCell.points[2],alpha: gridCell.values[1],beta: gridCell.values[2])
    }
    if (Tables.edgeTable[cubeindex] & 4) > 0 {
        vertexlist[2] = interpolateVertex(isolevel: isolevel,a: gridCell.points[2],b: gridCell.points[3],alpha: gridCell.values[2],beta: gridCell.values[3])
    }
    if (Tables.edgeTable[cubeindex] & 8) > 0 {
        vertexlist[3] = interpolateVertex(isolevel: isolevel,a: gridCell.points[3],b: gridCell.points[0],alpha: gridCell.values[3],beta: gridCell.values[0])
    }
    if (Tables.edgeTable[cubeindex] & 16) > 0 {
        vertexlist[4] = interpolateVertex(isolevel: isolevel,a: gridCell.points[4],b: gridCell.points[5],alpha: gridCell.values[4],beta: gridCell.values[5])
    }
    if (Tables.edgeTable[cubeindex] & 32) > 0 {
        vertexlist[5] = interpolateVertex(isolevel: isolevel,a: gridCell.points[5],b: gridCell.points[6],alpha: gridCell.values[5],beta: gridCell.values[6])
    }
    if (Tables.edgeTable[cubeindex] & 64) > 0 {
        vertexlist[6] = interpolateVertex(isolevel: isolevel,a: gridCell.points[6],b: gridCell.points[7],alpha: gridCell.values[6],beta: gridCell.values[7])
    }
    if (Tables.edgeTable[cubeindex] & 128) > 0 {
        vertexlist[7] = interpolateVertex(isolevel: isolevel,a: gridCell.points[7],b: gridCell.points[4],alpha: gridCell.values[7],beta: gridCell.values[4])
    }
    if (Tables.edgeTable[cubeindex] & 256) > 0 {
        vertexlist[8] = interpolateVertex(isolevel: isolevel,a: gridCell.points[0],b: gridCell.points[4],alpha: gridCell.values[0],beta: gridCell.values[4])
    }
    if (Tables.edgeTable[cubeindex] & 512) > 0 {
        vertexlist[9] = interpolateVertex(isolevel: isolevel,a: gridCell.points[1],b: gridCell.points[5],alpha: gridCell.values[1],beta: gridCell.values[5])
    }
    if (Tables.edgeTable[cubeindex] & 1024) > 0 {
        vertexlist[10] = interpolateVertex(isolevel: isolevel,a: gridCell.points[2],b: gridCell.points[6],alpha: gridCell.values[2],beta: gridCell.values[6])
    }
    if (Tables.edgeTable[cubeindex] & 2048) > 0 {
        vertexlist[11] = interpolateVertex(isolevel: isolevel,a: gridCell.points[3],b: gridCell.points[7],alpha: gridCell.values[3],beta: gridCell.values[7])
    }
    
    var i = 0
    while Tables.triTable[cubeindex][i] != -1 {
        var triangle = Triangle()
        let i1 = Tables.triTable[cubeindex][i+0]
        let i2 = Tables.triTable[cubeindex][i+1]
        let i3 = Tables.triTable[cubeindex][i+2]
        triangle.points[0] = vertexlist[Int(i1)]
        triangle.points[1] = vertexlist[Int(i2)]
        triangle.points[2] = vertexlist[Int(i3)]
        triangles.append(triangle)
        ntriangle += 1
        i += 3
    }
    return (ntriangle, triangles)
}
*/
