//
//  Box.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 20/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

/**
 * Box stands for a Cube.
 * Only two points are required : min and max.
 */
struct Box {
    var min: Vector // Lowest Vector (lexicographic order)
    var max: Vector // Highest Vector (lexicographic order)
    
    init() {
        min = Vector(0, 0, 0)
        max = Vector(1, 1, 1)
    }
    
    /**
     * Initializes a Box given two vectors.
     */
    init(min: Vector, max:Vector) {
        self.min = min
        self.max = max
    }
    
    /**
     * Get the absolute center of the box (centroid)
     */
    func getCenter() -> Vector {
        return 0.5 * (max + min)
    }
    
    /**
     * Get relative center of the box
     */
    func getExtent() -> Vector {
        return 0.5 * (max - min)
    }
    
    /**
     * Checks if the box has been initialized
     */
    func hasBeenInit() -> Bool {
        return (min == Vector(0, 0, 0) && max == Vector(1, 1, 1))
    }
}
