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
    
    var resolutionInMeter: Float   // Number of voxels per meter
    // Can size be only an Int ? Let's do this !
    //var size:       Point3D // X size, Y size and Z size
    var numberOfVoxels:   Int
    
    // Since i now the hash function, is adictionary still necessary ?
    //lazy var voxels:     [Int: Voxel]   = self.allocate()
    //lazy var centroids:  [Int: Vector]  = self.allocate()
    var voxels:    [Voxel]     //= self.allocate()
    //lazy var centroids: [Vector]    = self.allocate()
    //lazy var centroids: [Id3: Vector] = [Int: Vector]()
    
    private init() {
        numberOfVoxels      = 256
        resolutionInMeter   = 3.0
        voxels = [Voxel](repeating: Voxel(), count: Int(pow(Float(numberOfVoxels), 3.0)))
    }
    
    /*
    func allocate<T>() -> [T] {
        var allocator = [T]()
        allocator.removeAll()
        allocator.reserveCapacity(totalOfVoxels())
        return allocator
    }
    */
    
    func initialize() {
        /* Sequence - serial */
        let count = totalOfVoxels()
        //let square = size * size
        voxels.removeAll()
        voxels.reserveCapacity(totalOfVoxels())
        voxels = [Voxel](repeating: Voxel(), count: count)
        //centroids = [Vector](repeating: Point3D(0, 0, 0), count: count)
        
        /*
        let stride = MemoryLayout<Point3D>.stride
        let byteCount = stride * count
        let points = UnsafeMutablePointer<Point3D>.allocate(capacity: byteCount)
        bridge_initializeCentroids(points, Int32(numberOfVoxels), resolutionInMeter)
        let buffer = UnsafeBufferPointer(start: points, count: count)
        centroids = Array(buffer)
        points.deallocate()
        */
    }
    
    func initialize(with: Int) {
        numberOfVoxels = with
        initialize()
    }
    
    func totalOfVoxels() -> Int {
        return numberOfVoxels * numberOfVoxels * numberOfVoxels
    }
    
    func integrateDepthMap(image: DepthImage, camera: Camera, parameters: [Float]) {
        let width = image.width
        let height = image.height
        var dethmap = image.data
        var Rt = camera.extrinsics
        var K = camera.intrinsics
        let resolve = [resolutionInMeter, resolutionInMeter, resolutionInMeter]
        _ = bridge_integrateDepthMap(&dethmap, /*centroids,*/
            &Rt,
            &K,
            &voxels,
            Int32(width),
            Int32(height),
            Int32(numberOfVoxels),
            resolve,
            parameters[0], parameters[1], parameters[2]);
        //let voxel2 = voxels[numberOfVoxels].sdf
    }
    
    private func globalToInt(point: vector_float3) -> vector_float3{
        return Float(numberOfVoxels) * point / resolutionInMeter
    }
    private func intToGlobal(point: vector_int3) -> vector_float3 {
        let tmp = Vector(Float(point.x), Float(point.y), Float(point.z))
        return (resolutionInMeter / Float(numberOfVoxels)) * (tmp + 0.5);
    }
    
    private func updateVoxel(at: vector_int3, to: vector_float3) {
        let offset = resolutionInMeter / 2.0
        let i = hashed(point: at)
        if i < totalOfVoxels() && i >= 0 {
            let rpoint = intToGlobal(point: at) - offset
            let d = simd_length(rpoint - to)
            let z = rpoint.z
            
            voxels[i].update(sdf: d, weight: 1.0)
        }
    }
    
    private func hashed(point: vector_int3) -> Int {
    let a = Int(point.x) * (numberOfVoxels * numberOfVoxels)
    let b = Int(point.y) * numberOfVoxels
    let c = Int(point.z)
    return a + b + c
    }
    
    private func raycast(begin: Vector, end: Vector) {
        let offset = Float(resolutionInMeter / 2.0)
        let iStart = globalToInt(point: begin + offset)
        let iEnd = globalToInt(point: end + offset)
        
        var point = int3(Int32(iStart.x), Int32(iStart.y), Int32(iStart.z))
        let real_end =  int3(Int32(iEnd.x), Int32(iEnd.y), Int32(iEnd.z))
        
        //let x = point.x; let y=point.y; let z=point.z;
        //let sx = begin.x; let sy=begin.y; let sz=begin.z;
        
        var err_1:Int32 = 0
        var err_2:Int32 = 0
        let dx = real_end.x - point.x;
        let dy = real_end.y - point.y;
        let dz = real_end.z - point.z;
        let x_inc:Int32 = (dx < 0) ? -1 : 1;
        let l:Int32 = abs(dx);
        let y_inc:Int32 = (dy < 0) ? -1 : 1;
        let m:Int32 = abs(dy);
        let z_inc:Int32 = (dz < 0) ? -1 : 1;
        let n:Int32 = abs(dz);
        let dx2:Int32 = l << 1;
        let dy2:Int32 = m << 1;
        let dz2:Int32 = n << 1;
        if ((l >= m) && (l >= n)) {
            err_1 = dy2 - l;
            err_2 = dz2 - l;
            for _ in 0..<l {
                updateVoxel(at: point, to: end)
                if (err_1 > 0) {
                    point.y += y_inc;
                    err_1 -= dx2;
                }
                if (err_2 > 0) {
                    point.z += z_inc;
                    err_2 -= dx2;
                }
                err_1 += dy2;
                err_2 += dz2;
                point.x += x_inc;
            }
        } else if ((m >= l) && (m >= n)) {
            err_1 = dx2 - m;
            err_2 = dz2 - m;
            for _ in 0..<m {
                updateVoxel(at: point, to: end)
                if (err_1 > 0) {
                    point.x += x_inc;
                    err_1 -= dy2;
                }
                if (err_2 > 0) {
                    point.z += z_inc;
                    err_2 -= dy2;
                }
                err_1 += dx2;
                err_2 += dz2;
                point.y += y_inc;
            }
        } else {
            err_1 = dy2 - n;
            err_2 = dx2 - n;
            for _ in 0..<n {
                updateVoxel(at: point, to: end)
                if (err_1 > 0) {
                    point.y += y_inc;
                    err_1 -= dz2;
                }
                if (err_2 > 0) {
                    point.x += x_inc;
                    err_2 -= dz2;
                }
                err_1 += dy2;
                err_2 += dx2;
                point.z += z_inc;
            }
        }
        updateVoxel(at: point, to: end)
    }
    
    func integrate(points: ARPointCloud, camera: Camera) {
        //let n = numberOfVoxels
        //let n2 = numberOfVoxels * numberOfVoxels
        let offset = Float(resolutionInMeter / 2.0)
        let translation = Vector(camera.extrinsics.columns.3.x, camera.extrinsics.columns.3.y, camera.extrinsics.columns.3.z)
        for point in points.points {
            raycast(begin: Vector(0,0,0), end: point)
            //var tmp = point + offset
            //var vi = globalToInt(point: tmp)
            //var i = hashed(point: int3(Int32(0.5+vi.x), Int32(0.5+vi.y), Int32(0.5+vi.z)))
            //if i < totalOfVoxels() && i >= 0        { voxels[i].update(sdf: 0.0, weight: 1.0) }
            //tmp = (1 + resolutionInMeter / Float(numberOfVoxels)) * tmp
            //vi = globalToInt(point: tmp + offset)
            //i = hashed(point: int3(Int32(0.5+vi.x), Int32(0.5+vi.y), Int32(0.5+vi.z)))
            //if i < totalOfVoxels() && i >= 0        { voxels[i].update(sdf: 0.0, weight: 1.0) }
            /*if i-1 < totalOfVoxels() && i-1 >= 0    { voxels[i-1].update(sdf: 0.5, weight: 1.0) }
            //if i+1 < totalOfVoxels() && i+1 >= 0    { voxels[i+1].update(sdf: 0, weight: 1.0) }
            if i-n < totalOfVoxels() && i-n >= 0    { voxels[i-n].update(sdf: 0, weight: 1.0) }
            if i+n < totalOfVoxels() && i+n >= 0    { voxels[i+n].update(sdf: 0, weight: 1.0) }
            if i-n2 < totalOfVoxels() && i-n2 >= 0    { voxels[i-n2].update(sdf: 0, weight: 1.0) }
            if i+n2 < totalOfVoxels() && i+n2 >= 0    { voxels[i+n2].update(sdf: 0, weight: 1.0) }
             */
        }
    }
}

extension Vector {
    static func +(scalar: Float, vector: Vector) -> Vector {
        return Vector(scalar + vector.x, scalar + vector.y, scalar + vector.z)
    }
    static func +(vector: Vector, scalar: Float) -> Vector {
        return scalar + vector
    }
    static func -(scalar: Float, vector: Vector) -> Vector {
        return Vector(scalar - vector.x, scalar - vector.y, scalar - vector.z)
    }
    static func -(vector: Vector, scalar: Float) -> Vector {
        return scalar - vector
    }
}
