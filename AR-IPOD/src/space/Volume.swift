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
        size        = 64
        resolution  = 1.0
    }
    
    func allocate<T>() -> [T] {
        var allocator = [T]()
        allocator.removeAll()
        allocator.reserveCapacity(numberOfVoxels())
        return allocator
    }
    
    func initialize() {
        // TO DO : change 65536 by size dim
        /* When size is an (Float, Float, Float)
        let count = Int(size.x * size.y * size.z)
        let base2 = Int(size.x * size.x)
        let base = Int(size.x)
        */
        let count = numberOfVoxels()
        let square = size * size
        voxels = [Voxel](repeating: Voxel(), count: count)
        centroids = [Vector](repeating: Point3D(0, 0, 0), count: count)
        centroids = (0..<count).map {
                let x = Float($0 / square)
                let remainder = $0 % (square)
                let y = Float(remainder / size)
                let z = Float(remainder % size)
                return Point3D(x, y, z)
        }
        centroids = centroids.map { mappingIntegerToCentroid(point: $0, dim: size, voxelResolution: resolution) }
    }
    
    func numberOfVoxels() -> Int {
        //return Int(size.x * size.y * size.z)
        return size * size * size
    }
    
    func truncation(range: Float) -> Float {
        // TO DO
        return 1
    }
    
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
        let group = DispatchGroup()
        let _ = DispatchQueue.global(qos: .userInteractive)
        /*
        DispatchQueue.concurrentPerform(iterations: pointCloud.vertices.count) { i in
            let vertex = pointCloud.vertices[i]
            let depth = vertex.z
            if depth != 0.0 {
                group.enter()
                let id = mappingCentroidToInteger(centroid: vertex, dim: size, voxelResolution: resolution).index(base: size)
                let distance = (vertex - centroids[id]).length()
                voxels[id].update(sdfUpdate: distance, weightUpdate: 1)
                group.leave()
            }
        }
         */
        //for vertex in pointCloud.vertices {
        //let start = CFAbsoluteTimeGetCurrent()
        //while Double(CFAbsoluteTimeGetCurrent()) - Double(start) < 1.0 {
        let thread1 = DispatchQueue(label: "Thread1", qos: .userInteractive, attributes: .concurrent)
        let thread2 = DispatchQueue(label: "Thread2", qos: .userInteractive, attributes: .concurrent)
        let thread3 = DispatchQueue(label: "Thread3", qos: .userInteractive, attributes: .concurrent)
        /*
        for i in 0..<(pointCloud.vertices.count/3) {
            let j = Int(arc4random_uniform(UInt32(pointCloud.vertices.count)))
            let k = Int(arc4random_uniform(UInt32(pointCloud.vertices.count)))
            let m = Int(arc4random_uniform(UInt32(pointCloud.vertices.count)))
            let vertex1 = pointCloud.vertices[j]
            let vertex2 = pointCloud.vertices[k]
            let vertex3 = pointCloud.vertices[m]
            thread1.async {
                let depth = vertex1.z
                if depth != 0.0 {
                    let id = mappingCentroidToInteger(centroid: vertex1, dim: self.size, voxelResolution: self.resolution).index(base: self.size)
                    let distance = (vertex1 - self.centroids[id]).length()
                    self.voxels[id].update(sdfUpdate: distance, weightUpdate: 1)
                }
            }
            thread2.async {
                let depth = vertex2.z
                if depth != 0.0 {
                    let id = mappingCentroidToInteger(centroid: vertex2, dim: self.size, voxelResolution: self.resolution).index(base: self.size)
                    let distance = (vertex2 - self.centroids[id]).length()
                    self.voxels[id].update(sdfUpdate: distance, weightUpdate: 1)
                }
            }
            thread3.async {
                let depth = vertex3.z
                if depth != 0.0 {
                    let id = mappingCentroidToInteger(centroid: vertex3, dim: self.size, voxelResolution: self.resolution).index(base: self.size)
                    let distance = (vertex3 - self.centroids[id]).length()
                    self.voxels[id].update(sdfUpdate: distance, weightUpdate: 1)
                }
            }
        }
        DispatchQueue.main.sync {
            print("Done")
        }
        */
        //operationQueue.addOperations(operation1, waitUntilFinished: false)
        //}
        /*
        DispatchQueue.main.async {
            print("Integrate done")
        }
         */
    }
}
