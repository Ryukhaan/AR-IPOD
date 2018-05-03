//
//  VoxelSlice.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 19/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit
import AVFoundation

class Volume {
    // Singleton pattern : Only one volume will be create
    static let sharedInstance = Volume()
    
    static let tau_min: Float = 0.4
    
    var resolution: Float   // Number of voxels per meter
    // Can size be only an Int ? Let's do this !
    //var size:       Point3D // X size, Y size and Z size
    var size:   Int
    
    // Since i now the hash function, is adictionary still necessary ?
    //lazy var voxels:     [Int: Voxel]   = self.allocate()
    //lazy var centroids:  [Int: Vector]  = self.allocate()
    lazy var voxels:    [Voxel]     = self.allocate()
    lazy var centroids: [Vector]    = self.allocate()
    
    // Prevents others from using default init() for this class
    /*
    private init() {
        size        = Point3D(128, 128, 128)
        resolution  = 1.0
    }
    */
    
    private init() {
        size        = 128
        resolution  = 1.0
    }
    
    func allocate<T>() -> [T] {
        var allocator = [T]()
        allocator.removeAll()
        allocator.reserveCapacity(numberOfVoxels())
        return allocator
    }
    
    func initialize() {
        /* Sequence - serial */
        let count = numberOfVoxels()
        //let square = size * size
        voxels = [Voxel](repeating: Voxel(), count: count)
        centroids = [Vector](repeating: Point3D(0, 0, 0), count: count)
        
        let stride = MemoryLayout<Point3D>.stride
        let byteCount = stride * count
        let points = UnsafeMutablePointer<Point3D>.allocate(capacity: byteCount)
        bridge_initializeCentroids(points, Int32(size), resolution)
        let buffer = UnsafeBufferPointer(start: points, count: count)
        centroids = Array(buffer)
        points.deallocate()
    }
    
    func numberOfVoxels() -> Int {
        return size * size * size
    }
    
    func truncation(range: Float) -> Float {
        // TO DO
        return 1
    }
    
    /*
    func integrateDepthMap(image: DepthImage, camera: Camera) {
        // Get nearest and farthest depth
        let (minimumDepth, maximumDepth, _) = image.getStats()
        
        // Set near range and far range
        var copyCamera = camera
        copyCamera.zFar = maximumDepth
        copyCamera.zNear = minimumDepth
        
        // Create camera frustum
        var frustrum = Frustrum()
        frustrum.setUp(camera: copyCamera)
        
        // Determines intersects between frustrum and volume
        let bbox = computeBoundingBox(frustrum: frustrum)
        let voxelsIDs = retrieveIDs(from: bbox, dim: size, voxelResolution: resolution)
        
        // For each voxel/centroid retrieved
        let count = min(voxelsIDs.count, 2000)
        //let voxelGroup = DispatchGroup()
        //let _ = DispatchQueue.global(qos: .userInteractive)
        
        for i in 0..<count {
        //DispatchQueue.concurrentPerform(iterations: 2*count) { i in
            //voxelGroup.enter()
            //let start   = Double(CFAbsoluteTimeGetCurrent())
            let id = voxelsIDs[i]
            let centroid = self.centroids[id]
            let positionCamera = camera.extrinsics.columns.3
            let distance = (centroid - Vector(positionCamera.x, positionCamera.y, positionCamera.z)).length()
            let uv      = camera.project(vector: centroid)
            if (uv.x > Float(image.heigth) || uv.x < 0) { continue }
            if (uv.y > Float(image.width) || uv.y < 0) { continue }
            let depth   = image.at(row: Int(uv.x), column: Int(uv.y))
            if depth.isNaN { continue }
            if depth == 0.0 { continue }
            //if !depth.isNaN && depth != 0.0 {
                let proj    = camera.unproject(pixel: uv, depth: Float(depth))
                let range   = proj.length()
                let tau     = self.truncation(range: range)
                if distance >= Volume.tau_min && distance < tau - range {
                    self.voxels[id].carve()
                }
                else if fabs(distance) >= tau + range {
                    self.voxels[id].update(sdfUpdate: uv.x, weightUpdate: 1)
                }
            //}
            //voxelGroup.leave()
            //print("\( Double(CFAbsoluteTimeGetCurrent()) - start )")
        }
        //voxelGroup.wait()
    }
    */
    func integrateDepthMap(image: DepthImage, camera: Camera) {
        let width = image.width
        let height = image.height
        var K = camera.intrinsics
        var Rt = camera.extrinsics
        let dethmap = image.data
        _ = bridge_integrateDepthMap(dethmap, centroids, &Rt, &K, &voxels, Int32(width), Int32(height))
    }
    
    func randomSDF() {
        for _ in 0..<size*size {
            let j = Int(arc4random_uniform(UInt32(numberOfVoxels())))
            self.voxels[j].update(sdfUpdate: 0, weightUpdate: 1)
        }
    }
    
    func cuboid(at: Point3D) {
        let s2 = size*size
        let v0 = at.index(base: size)
        voxels[v0].update(sdfUpdate: 0, weightUpdate: 1)
        voxels[v0+1].update(sdfUpdate: 0, weightUpdate: 1)
        voxels[v0+size].update(sdfUpdate: 0, weightUpdate: 1)
        voxels[v0+size+1].update(sdfUpdate: 0, weightUpdate: 1)
        voxels[v0+s2].update(sdfUpdate: 0, weightUpdate: 1)
        voxels[v0+s2+1].update(sdfUpdate: 0, weightUpdate: 1)
        voxels[v0+s2+size].update(sdfUpdate: 0, weightUpdate: 1)
        voxels[v0+s2+size+1].update(sdfUpdate: 0, weightUpdate: 1)
    }
    
    func falseIntegration(pointCloud: PointCloud) {
        for (i, vertex) in pointCloud.vertices.enumerated() {
            if i > 1_000 { break }
            //let vertex = pointCloud.vertices[i]
            let depth = vertex.z
            if depth == 0.0 { continue }
            let id = mappingCentroidToInteger(centroid: vertex, dim: size, voxelResolution: resolution).index(base: size)
            let distance = (vertex - centroids[id]).length()
            voxels[id].update(sdfUpdate: distance, weightUpdate: 1)
        }
    }
}
