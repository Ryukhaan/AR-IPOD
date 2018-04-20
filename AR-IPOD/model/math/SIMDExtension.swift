//
//  SIMDExtension.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 20/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

extension float3 {
    func length() -> Float {
        return sqrtf(x*x + y*y + z*z)
    }
    
    /**
     * Normalizes the vector described by the float3 to length 1.0 and returns
     * the result as a new float3.
     */
    func normalized() -> float3 {
        return self / length()
    }
    
}
