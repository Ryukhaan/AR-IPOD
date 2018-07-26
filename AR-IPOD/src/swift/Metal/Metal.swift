//
//  Metal.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 26/07/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import Metal
import ARKit

func initMetal() -> (MTLDevice, MTLCommandQueue, MTLLibrary, MTLCommandBuffer,
    MTLComputeCommandEncoder) {
        // Get access to iPhone or iPad GPU
        let device = MTLCreateSystemDefaultDevice()
        
        // Queue to handle an ordered list of command buffers
        let commandQueue = device?.makeCommandQueue()
        
        // Access to Metal functions that are stored in Shaders.metal file, e.g. sigmoid()
        let defaultLibrary = device?.makeDefaultLibrary()
        
        // Buffer for storing encoded commands that are sent to GPU
        let commandBuffer = commandQueue?.makeCommandBuffer()
        
        // Encoder for GPU commands
        let computeCommandEncoder = commandBuffer?.makeComputeCommandEncoder()
        
        return (device!, commandQueue!, defaultLibrary!, commandBuffer!, computeCommandEncoder!)
}


