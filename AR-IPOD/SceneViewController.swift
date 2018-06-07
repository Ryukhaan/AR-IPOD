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
    
    var volume: Model = Model.sharedInstance
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

    @IBAction func update(_ sender: Any) {
        let iso = Float(isolevel.text!)
        DispatchQueue.global().async {
            let points = extractMesh(model: &self.volume, isolevel: iso!)
            //let pointCloudNode = createSimpleNode(from: volume, with: iso!)
            let pointCloudNode = UIFactory.createMeshNode(points: points)
            let scnView = self.view as! SCNView
            if self.self.indexOfPointCloud > 0 {
                scnView.scene?.rootNode.childNodes[self.indexOfPointCloud].removeFromParentNode()
            }
            scnView.scene?.rootNode.addChildNode(pointCloudNode)
            self.indexOfPointCloud = (scnView.scene?.rootNode.childNodes.count)! - 1
        }
        //scnView.backgroundColor = UIColor.black
    }
    
    @IBAction func displayPoints(_ sender: Any) {
        let iso = Float(withoutMesh.text!)
        DispatchQueue.global().async {
            //let points = extractMesh(volume: &self.volume, isolevel: iso!)
            let points = extractTSDF(model: self.volume, isolevel: iso!)
            //let pointCloudNode = createSimpleNode(from: volume, with: iso!)
            let pointCloudNode = UIFactory.createPointsNode(points: points)
            let scnView = self.view as! SCNView
            if self.self.indexOfPointCloud > 0 {
                scnView.scene?.rootNode.childNodes[self.indexOfPointCloud].removeFromParentNode()
            }
            scnView.scene?.rootNode.addChildNode(pointCloudNode)
            self.indexOfPointCloud = (scnView.scene?.rootNode.childNodes.count)! - 1
        }
    }
    @IBAction func export(_ sender: Any) {
        let iso = Float(isolevel.text!)
        let points = extractMesh(model: &self.volume, isolevel: iso!)
        exportToPLY(mesh: points, at: "meshing.ply")
    }
    
    @IBAction func display(_ sender: Any) {
        DispatchQueue.global().async {
            let points = extractMesh(model: &self.volume, isolevel: 0.02)
            //let pointCloudNode = createSimpleNode(from: volume, with: iso!)
            let pointCloudNode = UIFactory.createMeshNode(points: points)
            let scnView = self.view as! SCNView
            if self.self.indexOfPointCloud > 0 {
                scnView.scene?.rootNode.childNodes[self.indexOfPointCloud].removeFromParentNode()
            }
            scnView.scene?.rootNode.addChildNode(pointCloudNode)
            self.indexOfPointCloud = (scnView.scene?.rootNode.childNodes.count)! - 1
        }
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Release any cached data, images, etc that aren't in use.
    }
    
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        if segue.identifier == "Main" {
            if let destination = segue.destination as? ViewController {
                destination.myModel = self.volume
                destination.datasetChoice.selectedSegmentIndex  = savedDatasetIndex
                destination.datasetSize.selectedSegmentIndex    = savedFramesIndex
                destination.deltaStepper.value                  = savedDeltaIndex
                destination.epsilonStepper.value                = savedEpsilonIndex
                destination.stepperSize.value                   = savedVolumeSizeIndex
            }
        }
    }
}
