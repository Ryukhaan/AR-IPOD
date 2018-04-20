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
    var min: SCNVector3
    var max: SCNVector3
    
    init(min: SCNVector3, max:SCNVector3) {
        self.min = min
        self.max = max
    }
    
    func getCenter() -> SCNVector3 {
        return 0.5 * (max + min)
    }
    
    func getExtent() -> SCNVector3 {
        return 0.5 * (max - min)
    }
}
