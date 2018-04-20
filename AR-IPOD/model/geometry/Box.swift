//
//  Box.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 20/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

class Box {
    var min: Vector
    var max: Vector
    
    init() {
        min = Vector(0, 0, 0)
        max = Vector(1, 1, 1)
    }
    
    init(min: Vector, max:Vector) {
        self.min = min
        self.max = max
    }
    
    func getCenter() -> Vector {
        return 0.5 * (max + min)
    }
    
    func getExtent() -> Vector {
        return 0.5 * (max - min)
    }
}
