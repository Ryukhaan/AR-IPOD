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

func runMetal() -> [Float] {
    // Prepate vector
    var myvector = [Float](repeating: 0, count: 16_777_216)
    for (index, _) in myvector.enumerated() {
        myvector[index] = Float(index)
    }
    
    // a. initialize Metal
    let (device, _, defaultLibrary, commandBuffer, computeCommandEncoder) = initMetal()
    
    // b. set up a compute pipeline with Sigmoid function and add it to encoder
    let sigmoidProgram = defaultLibrary.makeFunction(name: "integrate")
    do {
        let computePipelineFilter = try device.makeComputePipelineState(function: sigmoidProgram!)
        computeCommandEncoder.setComputePipelineState(computePipelineFilter)
        // a. calculate byte length of input data - myvector
        let myvectorByteLength = myvector.count * MemoryLayout<Float>.stride
        
        // b. create a MTLBuffer - input data that the GPU and Metal and produce
        let inVectorBuffer = device.makeBuffer(bytes: &myvector, length: myvectorByteLength, options: .storageModeShared)
        
        // c. set the input vector for the Sigmoid() function, e.g. inVector
        //    atIndex: 0 here corresponds to buffer(0) in the Sigmoid function
        computeCommandEncoder.setBuffer(inVectorBuffer, offset: 0, index: 0)
        
        // d. create the output vector for the Sigmoid() function, e.g. outVector
        //    atIndex: 1 here corresponds to buffer(1) in the Sigmoid function
        var resultdata = [Float](repeating: 0, count: myvector.count)
        let outVectorBuffer = device.makeBuffer(bytes: &resultdata, length: myvectorByteLength, options: .storageModeShared)
        computeCommandEncoder.setBuffer(outVectorBuffer, offset: 0, index: 1)
        
        // hardcoded for now (recommendation: read about threadExecutionWidth)
        let threadsPerThreadgroup = MTLSizeMake(16, 8, 8)
        let threadsPerGrid = MTLSize(width: 16, height:32, depth:32)
        
        computeCommandEncoder.dispatchThreadgroups(threadsPerGrid, threadsPerThreadgroup: threadsPerThreadgroup)
        computeCommandEncoder.endEncoding()
        commandBuffer.commit()
        commandBuffer.waitUntilCompleted()
        // a. Get GPU data
        // outVectorBuffer.contents() returns UnsafeMutablePointer roughly equivalent to char* in C
        let data = NSData(bytesNoCopy: (outVectorBuffer?.contents())!,
                          length: myvector.count * MemoryLayout<Float>.stride,
                          freeWhenDone: false)
        // b. prepare Swift array large enough to receive data from GPU
        var finalResultArray = [Float](repeating: 0, count: myvector.count)
        
        // c. get data from GPU into Swift array
        data.getBytes(&finalResultArray, length: myvector.count * MemoryLayout<Float>.stride)
        
        // d. YOU'RE ALL SET!
        return finalResultArray
    }
    catch { return [Float]() }
}

