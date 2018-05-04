//
//  Cell.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 26/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

struct Cell {
    var points: [Vector] = [Vector].init(repeating: Vector(0,0,0), count: 8)
    var values: [Float]  = [Float].init(repeating: 0.0, count: 8)
    
    init(_points: [Vector], _values: [Float]) {
        points = _points
        values = _values
    }
}
