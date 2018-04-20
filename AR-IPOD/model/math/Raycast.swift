//
//  Raycast.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 20/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

/**
 * Return 0, 1 or -1 according to the sign of x
 */
func signed(x: Int) -> Int {
    return x == 0 ? 0 : x > 0 ? 1 : -1
}

func mod(x: Float, modulus: Float) -> Float {
    return fmod(fmod(x, modulus) + modulus, modulus)
}

/**
 * Finds the smallest t such s + t * d is an Integer
 */
func intBound(s: Float, d: Float) -> Float {
    if d < 0 {
        return intBound(s: -s, d: -d)
    }
    let ds = mod(x: s, modulus: 1)
    return (1 - ds) / d
}

/**
 * @param : minimal is the minimal pointel of Box (left-bottom)
 * @param : maximal is the maximal pointel of Box (rigth-top)
 */
func rayIntersectsBox(start: SCNVector3,
                      end: SCNVector3,
                      minimal: SCNVector3,
                      maximal: SCNVector3) -> Bool{
    let direction       = (start - end).normalized()
    let directionInv    = SCNVector3Make(1.0 / direction.x, 1.0 / direction.y, 1.0 / direction.z)
    
    let t1 = (minimal.x - start.x) * directionInv.x;
    let t2 = (maximal.x - start.x) * directionInv.x;
    let t3 = (minimal.y - start.y) * directionInv.y;
    let t4 = (maximal.y - start.y) * directionInv.y;
    let t5 = (minimal.z - start.z) * directionInv.z;
    let t6 = (maximal.z - start.z) * directionInv.z;
    
    let tmin = fmax(fmax(fmin(t1, t2), fmin(t3, t4)), fmin(t5, t6));
    let tmax = fmin(fmin(fmax(t1, t2), fmax(t3, t4)), fmax(t5, t6));
    
    // if tmax < 0, ray (line) is intersecting AABB, but whole AABB is behing us
    if (tmax < 0) { return false }
    // if tmin > tmax, ray doesn't intersect AABB
    if (tmin > tmax) { return false }
    return true;
}

/**
 * Raycasting
 * From "A Fast Voxel Traversal Algorithm for Ray Tracing"
 * by John Amanatides and Andrew Woo, 1987
 * <http://www.cse.yorku.ca/~amana/research/grid.pdf>
 * <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
 * Extensions to the described algorithm:
 *      Imposed a distance limit.
 *      The face passed through to reach the current cube is provided to the callback.
 *
 * The foundation of this algorithm is a parameterized representation of
 * the provided ray :
 *                    origin + t * direction,
 * except that t is not actually stored; rather, at any given point in the
 * traversal, we keep track of the *greater* t values which we would have
 * if we took a step sufficient to cross a cube boundary along that axis
 * (i.e. change the integer part of the coordinate) in the variables
 * tMaxX, tMaxY, and tMaxZ.
 */
func raycast(start: float3,
             end: float3,
             min: int3,
             max: int3,
             output: inout [int3]) {
    // Create Interger cube
    var x = Int32(floor(start.x))
    var y = Int32(floor(start.y))
    var z = Int32(floor(start.z))
    let endX = Int32(floor(end.x))
    let endY = Int32(floor(end.y))
    let endZ = Int32(floor(end.z))
    let direction = (end - start)
    let maxDistance = direction.length()
    
    // Break out direction vector
    let dx = Float(endX - x)
    let dy = Float(endY - y)
    let dz = Float(endZ - z)
    
    // Step increments
    let stepX = Int32(signed(x: Int(dx)))
    let stepY = Int32(signed(x: Int(dy)))
    let stepZ = Int32(signed(x: Int(dz)))
    
    // See description above. The initial values depend on the fractional part of the origin.
    var tMaxX = intBound(s: start.x, d: dx)
    var tMaxY = intBound(s: start.y, d: dy)
    var tMaxZ = intBound(s: start.z, d: dz)
    
    // The change in t when taking a step (always positive).
    let tDeltaX = Float(stepX) / dx
    let tDeltaY = Float(stepY) / dy
    let tDeltaZ = Float(stepZ) / dz
    
    // No infinite loop
    if (stepX == 0 && stepY == 0 && stepZ == 0) { return }
    
    var distance = 0
    // Maximum number of voxels
    let thresh = 1500
    while true
    {
        if (x >= Int(min.x) && x < Int(max.x)
            && y >= Int(min.y) && y < Int(max.y)
            && z >= Int(min.z) && y < Int(max.z))
        {
            let point = int3(x, y, z)
            output.append(point)
            distance = Int((float3(Float(x), Float(y), Float(z)) - start).length())
            if (Float(distance) > maxDistance) { return }
            assert(output.count < thresh, "Too many voxels")
        }
        // End of the raycasting
        if (x == endX && y == endY && z == endZ) { break }
        if (tMaxX < tMaxY) {
            if (tMaxX < tMaxZ) {
                // Update which cube we are now in.
                x += stepX
                // Adjust tMaxX to the next X-oriented boundary crossing.
                tMaxX += tDeltaX
            }
            else {
                z += stepZ
                tMaxZ += tDeltaZ
            }
        }
        else {
            if (tMaxY < tMaxZ) {
                y += stepY
                tMaxY += tDeltaY
            }
            else {
                z += stepZ
                tMaxZ += tDeltaZ
            }
        }
    }
}
