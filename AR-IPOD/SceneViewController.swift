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
    @IBOutlet var withoutMesh: UITextField!
    
    @IBOutlet var wireFrame: UISwitch!
    
    var myModel: Model = Model.sharedInstance
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

    @IBAction func meshWithIso(_ sender: Any) {
        let iso = Float(isolevel.text!)
        DispatchQueue.global().async {
            let points = extractMesh(model: Model.sharedInstance, isolevel: iso!)
            let pointCloudNode = UIFactory.mesh(from: points)
            let scnView = self.view as! SCNView
            if self.self.indexOfPointCloud > 0 {
                scnView.scene?.rootNode.childNodes[self.indexOfPointCloud].removeFromParentNode()
            }
            scnView.scene?.rootNode.addChildNode(pointCloudNode)
            self.indexOfPointCloud = (scnView.scene?.rootNode.childNodes.count)! - 1
        }
    }
    
    @IBAction func showOnlyPoints(_ sender: Any) {
        //let iso = Int(withoutMesh.text!)
        let iso = Float(withoutMesh.text!)
        let datasetName = "ikea-table"
        DispatchQueue.global().async {
            // Display Projected DepthMap
            /*
            Model.sharedInstance.switchTo(type: .Kinect)
            let intrinsics = Import.intrinsics(from: "depthIntrinsics", at: datasetName, type: Model.sharedInstance.type)
            Model.sharedInstance.update(intrinsics: intrinsics)
            let depthmap = Import.depthMapFromTXT(from: "frame-\(iso!).depth",at: datasetName, type: Model.sharedInstance.type)
            Model.sharedInstance.update(data: depthmap)
            let points = projectDepthMap(from: Model.sharedInstance)
            */
            
            // Display SDF without mesh
            let points = extractTSDF(model: Model.sharedInstance, isolevel: iso!)
            let pointCloudNode = UIFactory.pointCloud(points: points)
            let scnView = self.view as! SCNView
            if self.self.indexOfPointCloud > 0 {
                scnView.scene?.rootNode.childNodes[self.indexOfPointCloud].removeFromParentNode()
            }
            scnView.scene?.rootNode.addChildNode(pointCloudNode)
            self.indexOfPointCloud = (scnView.scene?.rootNode.childNodes.count)! - 1
        }
    }
    
    @IBAction func export(_ sender: Any) {
        // Export mesh at an isolovel (need to extract mesh once again, not so logical but easiest way)
        let iso = Float(isolevel.text!)
        let points = extractMesh(model: Model.sharedInstance, isolevel: iso!)
        exportToPLY(mesh: points, at: "meshing.ply")
    }
    
    @IBAction func fastMeshing(_ sender: Any) {
        // On IphoneX keyboard won't close (so meshWithIso() does not work)
        // Extract mesh with isolevel = 0.0
        DispatchQueue.global().async {
            let points = extractMesh(model: Model.sharedInstance, isolevel: 0.00)
            let pointCloudNode = UIFactory.mesh(from: points)
            let scnView = self.view as! SCNView
            if self.self.indexOfPointCloud > 0 {
                scnView.scene?.rootNode.childNodes[self.indexOfPointCloud].removeFromParentNode()
            }
            scnView.scene?.rootNode.addChildNode(pointCloudNode)
            self.indexOfPointCloud = (scnView.scene?.rootNode.childNodes.count)! - 1
        }
    }
    
    @IBAction func showWireFrame(_ sender: Any) {
        // Switch between wireframe mode and texture mode
        let scnView = self.view as! SCNView
        if wireFrame.isOn {
            scnView.scene?.rootNode.childNodes[indexOfPointCloud].geometry?.firstMaterial?.fillMode = .lines
        }
        else {
            scnView.scene?.rootNode.childNodes[indexOfPointCloud].geometry?.firstMaterial?.fillMode = .fill
        }
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Release any cached data, images, etc that aren't in use.
    }
    
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        if segue.identifier == "Main" {
            if let destination = segue.destination as? ViewController {
                destination.myModel = Model.sharedInstance
                destination.datasetChoice.selectedSegmentIndex  = savedDatasetIndex
                destination.datasetSize.selectedSegmentIndex    = savedFramesIndex
            }
        }
    }
}
