//
//  Geometry.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 20/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

/**
 * Different intersection types (mostly for plane) :
 * - Inside     : a object lies inside another one
 * - Outise     : a object lies outside another one
 * - Intersect  : a object intersects another one
 */
enum IntersectType {
    case inside
    case outside
    case intersect
}

/**
 * BOXES PREDICATES
 */

/**
 * Determines if a Box intersects another one.
 */
func intersects(first: Box, second: Box) -> Bool {
    if (first.min.x > second.max.x) { return false }
    if (first.min.y > second.max.y) { return false }
    if (first.min.z > second.max.z) { return false }
    if (first.max.x < second.min.x) { return false }
    if (first.max.x < second.min.x) { return false }
    if (first.max.x < second.min.x) { return false }
    return true
}

/**
 * Determines intersection type between a Box and a Plane.
 */
func intersects(box: Box, plane: Plane) -> IntersectType {
    // We need to check every pointels
    let extent = box.getExtent()
    var corners = [Vector](repeating: Vector(0, 0, 0), count: 8)
    corners[0] = box.max
    corners[1] = box.min
    corners[2] = box.min + Vector(extent.x, 0, 0)
    corners[3] = box.min + Vector(0, extent.y, 0)
    corners[4] = box.min + Vector(0, 0, extent.z)
    corners[5] = box.min + Vector(extent.x, 0, extent.z)
    corners[6] = box.min + Vector(extent.x, extent.y, 0)
    corners[7] = box.min + Vector(0, extent.y, extent.z)
    
    var lastDistance = dot(plane.normal, corners[0]) + plane.distance
    // For each corner, if there is at least one change of sign then it intersects the box
    for i in 1..<8 {
        let distance = dot(plane.normal, corners[i]) + plane.distance
        if ((distance <= 0 && lastDistance > 0)
            || (distance >= 0 && lastDistance < 0)) {
            return .intersect
        }
        lastDistance = distance;
    }
    // If all distances from a corner to the plane are the same, look only at the last one
    if (lastDistance > 0) {
        return .outside
    }
    return .inside
}

/**
 * Determines if Box contains strictly Point.
 */
func contains(box: Box, point: SCNVector3) -> Bool {
    return point.x >= box.min.x && point.y >= box.min.y && point.z >= box.min.z
        && point.x <= box.max.x && point.y <= box.max.y && point.z <= box.max.z
}

/**
 * PLANES PREDICATES
 */

/**
 * Calculates signed distance between a plane and a point.
 */
func signedDistance(from: Plane, to: Vector) -> Float {
    return dot(from.normal, to) + from.distance
}

/**
 * Determines intersection type between a plane and a point.
 */
func intersects(plane: Plane, vector: Vector) -> IntersectType {
    let d = signedDistance(from: plane, to: vector)
    if (d < 0) {
        return .inside
    }
    else if (d > 0) {
        return .outside
    }
    return .intersect
}

/**
 * FRUSTRUM PREDICATES
 */

/**
 * Determines if Box intersects Frustum.
 */
func intersects(frustrum: Frustrum, box: Box) -> Bool {
    let planes = frustrum.getPlanes()
    
    for plane in planes {
        var axis = Vector(0, 0, 0)
        let normal = plane.normal;
        // x-axis
        // Which Box pointel is furthest down (plane normals direction) the x axis.
        if (normal.x < 0.0) { axis.x = box.min.x }
        else { axis.x = box.max.x }
        
        // y-axis
        // Which Box pointel is furthest down (plane normals direction) the y axis.
        if (normal.y < 0.0) { axis.y = box.min.y }
        else { axis.y = box.max.y }
        
        // z-axis
        // Which Box pointel is furthest down (plane normals direction) the z axis.
        if (normal.z < 0.0) { axis.z = box.min.z }
        else { axis.z = box.max.z }
        
        // Now we get the signed distance from the Box pointel that's furthest down
        // the frustum planes normal, and if the signed distance is negative, then
        // the entire bounding box is behind the frustum plane.
        if ( (dot(axis, normal) + plane.distance) > 0.0) {
            return true
        }
    }
    return true
}

/**
 * Determines if Frustrum contains strictly the Point.
 */
func contains(frustrum: Frustrum, point: Vector) -> Bool {
    // What if frustrum's planes normals are misgenerated ?
    let planes = frustrum.getPlanes()
    for plane in planes {
        if intersects(plane: plane, vector: point) == .outside {
            return false
        }
    }
    // Normally frustum is convex intersection of positive part of 6 planes.
    // So, it is inside the frustrum if the point is inside all the planes.
    return true
}


/**
 * GEOMETRIC FUNCTIONS
 */

/**
 * Compute a Box given a Frustum
 */
func computeBoundingBox(frustrum: Frustrum) -> Box {
    let bigNum = Float(Int32.max)
    var tmpMin = Vector(-bigNum, -bigNum, -bigNum)
    var tmpMax = Vector(bigNum, bigNum, bigNum)
    for pointel in frustrum.pointels {
        tmpMin.x = min(tmpMin.x, pointel.x)
        tmpMin.y = min(tmpMin.y, pointel.y)
        tmpMin.z = min(tmpMin.z, pointel.z)
        tmpMax.x = max(tmpMax.x, pointel.x)
        tmpMax.y = max(tmpMax.y, pointel.y)
        tmpMax.z = max(tmpMax.z, pointel.z)
    }
    return Box(min: tmpMin, max: tmpMax)
}

/**
 * Project a vector to image plane given a matrix K.
 * Projection between 3D and 2D.
 */
func project(vector: Vector, K: matrix_float3x3) -> Pixel {
    let temp = Vector(vector.x / vector.z, vector.y / vector.z, 1)
    let all  = K * temp
    return Pixel(all.x, all.y)
}

/**
 * Unproject a pixel to the 3D world coordinate given a matrix K.
 * (Un)Projection between 2D and 3D.
 */
func unproject(pixel: Pixel, depth: Float, K: matrix_float3x3) -> Vector {
    let temp = Point3D(pixel.x, pixel.y, 1)
    let all = K.inverse * temp
    return Vector(all.x * depth, all.y * depth, depth)
}

/**
 * Unproject a pixel to the 3D world coordinate given a matrix K.
 * (Un)Projection between 2D and 3D.
 */
func unproject(vector: Vector, K: matrix_float3x3) -> Vector {
    let temp = Point3D(vector.x, vector.y, 1)
    let all = K.inverse * temp
    return Vector(all.x * vector.z, all.y * vector.z, vector.z)
}

/**
 * Mapping between a world point and a voxel coordinate
 * Step is a Float, but in pratical it will be an Integer.
 */
func mappingVoxel(worldPoint: Vector, dim: Point3D, step: Float) -> Point3D {
    return step * Point3D(worldPoint.x / dim.x + 0.5,
                   worldPoint.y / dim.y + 0.5,
                   worldPoint.z / dim.z + 0.5)
}

/**
 * Computes linear interpolation.
 */
func linearInterpolate(u: Point3D, v: Point3D, tx: Float) -> Point3D {
    return tx * u + (1 - tx) * v
}

/**
 * Computes bilinear interpolation.
 */
func bilinearInterpolate(c00: Point3D, c01: Point3D, c10: Point3D, c11: Point3D, tx: Float, ty: Float) -> Point3D {
    return linearInterpolate(u: linearInterpolate(u: c00, v: c10, tx: tx),
                             v: linearInterpolate(u: c01, v: c11, tx: tx),
                             tx: ty)
}

/**
 * Computes trilinear interpolation with linear and bilinear interpolation.
 */
func trilinearInterpolate(position: Point3D) -> Vector {
    let gp = Vector(floor(position.x), floor(position.y), floor(position.z))
    let (tx, ty, tz) = (position.x - gp.x, position.y - gp.y, position.z - gp.z)
    let c000 = Vector(gp.x, gp.y, gp.z)
    let c100 = Vector(gp.x+1, gp.y, gp.z)
    let c010 = Vector(gp.x, gp.y+1, gp.z)
    let c110 = Vector(gp.x+1, gp.y+1, gp.z)
    let c001 = Vector(gp.x, gp.y, gp.z+1)
    let c101 = Vector(gp.x+1, gp.y, gp.z+1)
    let c011 = Vector(gp.x, gp.y+1, gp.z+1)
    let c111 = Vector(gp.x+1, gp.y+1, gp.z+1)
    let e = bilinearInterpolate(c00: c000, c01: c100, c10: c010, c11: c110, tx: tx, ty: ty)
    let f = bilinearInterpolate(c00: c001, c01: c101, c10: c011, c11: c111, tx: tx, ty: ty)
    return linearInterpolate(u: e, v: f, tx: tz)
}

/**
 * From global coordinate to local
 */
func globalToLocal(worldPoint: Vector, Rt: matrix_float4x4) -> Vector {
    let truncation = matrix_float4x3(rows: [Rt[0], Rt[1], Rt[2]])
    let rotation = matrix_float3x3(truncation.columns.0, truncation.columns.1, truncation.columns.2)
    let translation = truncation.columns.3
    return rotation * (worldPoint - translation)
}

/**
 * Retrives all ID in a certain box
 */
func retriveIDs(from: Box, dim: Point3D, step: Float) -> [Int] {
    var list = [Int]()
    let mini = from.min
    let maxi = from.max
    for x in Int(mini.x)...Int(maxi.x) {
        for y in Int(mini.y)...Int(maxi.y) {
            for z in Int(mini.z)...Int(maxi.z) {
                let worldPoint  = Vector(Float(x), Float(y), Float(z))
                let approx      = mappingVoxel(worldPoint: worldPoint, dim: dim, step: step)
                let nindex      = trilinearInterpolate(position: approx).index()
                list.append(nindex)
            }
        }
    }
    return list
}
