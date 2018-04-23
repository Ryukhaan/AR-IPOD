//
//  Frustrum.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 20/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

class Frustrum {
    var top: Plane
    var left: Plane
    var bottom: Plane
    var right: Plane
    var near: Plane
    var far: Plane
    var pointels    = [Vector](repeating: Vector(0,0,0), count: 8)
    var linels      = [Int](repeating: 0, count: 24)
    
    init() {
        top     = Plane()
        bottom  = Plane()
        left    = Plane()
        right   = Plane()
        near    = Plane()
        far     = Plane()
    }
    
    func setUpFromVectors(forward: Vector,
                          pos: Vector,
                          rightVec: Vector,
                          up: Vector,
                          nearDist: Float,
                          farDist: Float,
                          fov: Float,
                          aspect: Float) {
        
        let angleTangent = tan(fov / 2)
        let heightFar = angleTangent * farDist
        let widthFar = heightFar * aspect
        let heightNear = angleTangent * nearDist
        let widthNear = heightNear * aspect
        let farCenter = pos + farDist * forward
        let farTopLeft = farCenter + (heightFar * up ) - (widthFar * rightVec)
        let farTopRight = farCenter + (heightFar * up ) + (widthFar * rightVec)
        let farBotLeft = farCenter - (heightFar * up) - (widthFar * rightVec)
        let farBotRight = farCenter - (heightFar * up) + (widthFar * rightVec)
        
        let nearCenter = pos + nearDist * forward
        let nearTopLeft = nearCenter + (heightNear * up) - (widthNear * rightVec)
        let nearTopRight = nearCenter + (heightNear * up) + (widthNear * rightVec)
        let nearBotLeft = nearCenter - (heightNear * up) - (widthNear * rightVec)
        let nearBotRight = nearCenter - (heightNear * up) + (widthNear * rightVec)
        
        near = Plane(nearBotLeft, nearTopLeft, nearBotRight)
        far = Plane(farTopRight, farTopLeft, farBotRight)
        left = Plane(farTopLeft, nearTopLeft, farBotLeft)
        right = Plane(nearTopRight, farTopRight, nearBotRight)
        top = Plane(nearTopLeft, farTopLeft, nearTopRight)
        bottom = Plane(nearBotRight, farBotLeft, nearBotLeft)
        
        pointels[0] = farTopLeft
        pointels[1] = farTopRight
        pointels[2] = farBotRight
        pointels[3] = farBotLeft
        pointels[4] = nearTopLeft
        pointels[5] = nearTopRight
        pointels[6] = nearBotRight
        pointels[7] = nearBotLeft
        
        // Far and near face linels.
        for i in 0..<4 {
            // Far face
            linels[2*i]     = i
            linels[2*i+1]   = (i + 1) % 4
            // Near face
            linels[2*i+8]   = i + 4
            linels[2*i+9]   = (i + 1) % 4 + 4
            // Connecting linels
            linels[2*i+16]  = i
            linels[2*i+17]  = i + 4
        }
    }
    
    /**
     * Returns an array with all planes.
     */
    func getPlanes() -> [Plane] {
        return [Plane](arrayLiteral: far, near, top, bottom, left, right)
    }
    
    /**
     * Set up Frustum given a camra
     */
    func setUp(camera: Camera) {
        let rot         = camera.extrinsics
        let right       = Vector(rot[0][0], rot[1][0], rot[2][0])
        let up          = -Vector(rot[0][1], rot[1][1], rot[2][1])
        let d           = Vector(rot[0][2], rot[1][2], rot[2][2])
        let position    = Vector(rot[0][3], rot[1][3],rot[2][3])
        let fx          = camera.intrinsics[0][0]
        let fy          = camera.intrinsics[1][1]
        let cy          = camera.intrinsics[1][2]
        let aspect = (fx * Float(camera.width)) / (fy * Float(camera.height));
        let fov = atan2(cy, fy) + atan2(Float(camera.height) - cy, fy);
        setUpFromVectors(forward: d,
                         pos: position,
                         rightVec: right,
                         up: up,
                         nearDist: Float(camera.zNear), farDist: Float(camera.zFar),
                         fov: fov, aspect: aspect)
    }
}
