//
//  SceneViewController.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 11/05/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import SceneKit


class SceneViewController: UIViewController {
    //@IBOutlet var sceneView: SCNView!
    
    @IBOutlet var isoLabel: UILabel!
    @IBOutlet var isolevel: UITextField!
    
    var volume: Volume = Volume.sharedInstance
    var indexOfPointCloud: Int = 0
    var savedDeltaIndex: Double = 0
    var savedEpsilonIndex: Double = 0
    var savedVolumeSizeIndex: Double = 0
    var savedDatasetIndex: Int = 0
    var savedFramesIndex: Int = 0
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // create a new scene
        let scene = SCNScene()
        
        // create and add a camera to the scene
        let cameraNode = SCNNode()
        cameraNode.camera = SCNCamera()
        cameraNode.position = SCNVector3(x: 0, y: 0, z: 5)
        cameraNode.camera?.zNear = 0.01
        cameraNode.camera?.zFar = 15
        scene.rootNode.addChildNode(cameraNode)
    
        
        // create and add a light to the scene
        
        // add a cube
        let cube = SCNBox(width: 0.1, height: 0.1, length: 0.1, chamferRadius: 0)
        let node = SCNNode(geometry: cube)
        scene.rootNode.addChildNode(node)
        indexOfPointCloud = scene.rootNode.childNodes.count - 1
        
        // create View
        let sceneView = view as! SCNView
        //sceneView = SCNView(frame: CGRect(x: 0, y: 0, width: 300, height: 300))
        
        // update scene
        sceneView.scene = scene
        
        // autolights the scene
        sceneView.autoenablesDefaultLighting = true
        
        // allows the user to manipulate the camera
        sceneView.allowsCameraControl = true
        
        // show statistics such as fps and timing information
        sceneView.showsStatistics = false
        
        // configure the view
        sceneView.backgroundColor = UIColor.lightGray
        //view.backgroundColor = UIColor.lightGray
        
        // What is that ?
        sceneView.rendersContinuously = true
    }

    @IBAction func update(_ sender: Any) {
        let iso = Float(isolevel.text!)
        isoLabel.text = "Isolevel: \(String(describing: iso!))"
        let points = extractMesh(volume: volume, isolevel: iso!)
        //let pointCloudNode = createSimpleNode(from: volume, with: iso!)
        let pointCloudNode = createSimpleNode(from: points)
        let scnView = view as! SCNView
        if indexOfPointCloud > 0 {
            scnView.scene?.rootNode.childNodes[indexOfPointCloud].removeFromParentNode()
        }
        scnView.scene?.rootNode.addChildNode(pointCloudNode)
        indexOfPointCloud = (scnView.scene?.rootNode.childNodes.count)! - 1
        //scnView.backgroundColor = UIColor.black
    }
    
    func createSimpleNode(from: Volume, with: Float) -> SCNNode{
        var points = [Vector]()
        let size = from.totalOfVoxels()
        for i in 0..<size {
            if (abs(from.voxels[i].sdf) <= with) {
                points.append(from.centroids[i])
            }
        }
        let vertexData = NSData(bytes: points, length: MemoryLayout<Vector>.stride * points.count)
        let positionSources = SCNGeometrySource(data: vertexData as Data,
                                                semantic: SCNGeometrySource.Semantic.vertex,
                                                vectorCount: points.count,
                                                usesFloatComponents: true,
                                                componentsPerVector: 3,
                                                bytesPerComponent: MemoryLayout<Float>.size,
                                                dataOffset: 0,
                                                dataStride: MemoryLayout<Vector>.stride)
        let elements = SCNGeometryElement(
            data: nil,
            primitiveType: .point,
            primitiveCount: points.count,
            bytesPerIndex: MemoryLayout<Double>.size
        )
        let pointCloudGeometry = SCNGeometry(sources: [positionSources], elements: [elements])
        return SCNNode(geometry: pointCloudGeometry)
    }
    
    func createSimpleNode(from: [Vector]) -> SCNNode{
        let vertexData = NSData(bytes: from, length: MemoryLayout<Vector>.stride * from.count)
        let positionSources = SCNGeometrySource(data: vertexData as Data,
                                                semantic: SCNGeometrySource.Semantic.vertex,
                                                vectorCount: from.count,
                                                usesFloatComponents: true,
                                                componentsPerVector: 3,
                                                bytesPerComponent: MemoryLayout<Float>.size,
                                                dataOffset: 0,
                                                dataStride: MemoryLayout<Vector>.stride)
        let elements = SCNGeometryElement(
            data: nil,
            primitiveType: .triangles,
            primitiveCount: from.count / 3,
            bytesPerIndex: MemoryLayout<Double>.size
        )
        let pointCloudGeometry = SCNGeometry(sources: [positionSources], elements: [elements])
        return SCNNode(geometry: pointCloudGeometry)
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Release any cached data, images, etc that aren't in use.
    }
    
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        if segue.identifier == "Main" {
            if let destination = segue.destination as? ViewController {
                //destination.myVolume = self.volume
                destination.datasetChoice.selectedSegmentIndex  = savedDatasetIndex
                destination.datasetSize.selectedSegmentIndex    = savedFramesIndex
                destination.deltaStepper.value                  = savedDeltaIndex
                destination.epsilonStepper.value                = savedEpsilonIndex
                destination.stepperSize.value                   = savedVolumeSizeIndex
            }
        }
    }
}
