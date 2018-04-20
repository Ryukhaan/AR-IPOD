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
    var corners = [SCNVector3](repeating: SCNVector3Make(0, 0, 0), count: 8)
    corners[0] = box.max
    corners[1] = box.min
    corners[2] = box.min + SCNVector3Make(extent.x, 0, 0)
    corners[3] = box.min + SCNVector3Make(0, extent.y, 0)
    corners[4] = box.min + SCNVector3Make(0, 0, extent.z)
    corners[5] = box.min + SCNVector3Make(extent.x, 0, extent.z)
    corners[6] = box.min + SCNVector3Make(extent.x, extent.y, 0)
    corners[7] = box.min + SCNVector3Make(0, extent.y, extent.z)
    
    var lastDistance = SCNVector3.dot(u: plane.normal, v: corners[0]) + plane.distance
    // For each corner, if there is at least one change of sign then it intersects the box
    for i in 1..<8 {
        let distance = SCNVector3.dot(u: plane.normal, v: corners[i]) + plane.distance
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
func signedDistance(from: Plane, to: SCNVector3) -> Float {
    return SCNVector3.dot(u: from.normal, v: to) + from.distance
}

/**
 * Determines intersection type between a plane and a point.
 */
func intersects(plane: Plane, vector: SCNVector3) -> IntersectType {
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
        var axis = SCNVector3(0, 0, 0)
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
        if ( (SCNVector3.dot(u: axis, v:normal) + plane.distance) > 0.0) {
            return true
        }
    }
    return true
}

/**
 * Determines if Frustrum contains strictly the Point.
 */
func contains(frustrum: Frustrum, point: SCNVector3) -> Bool {
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


