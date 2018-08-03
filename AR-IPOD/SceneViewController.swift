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
    
    //@IBOutlet var isoLabel: UILabel!
    //@IBOutlet var isolevel: UITextField!
    //@IBOutlet var withoutMesh: UITextField!
    
    @IBOutlet var wireFrame: UISwitch!
    
    var model: Model = Model.sharedInstance
    var meshIndex: Int = 0
    //var savedDeltaIndex: Double = 0
    //var savedEpsilonIndex: Double = 0
    //var savedVolumeSizeIndex: Double = 0
    //var savedDatasetIndex: Int = 0
    //var savedFramesIndex: Int = 0
    var points: [Vector] = [Vector]()
    var isMeshed: Bool = false

    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // create a new scene
        let scene = SCNScene()
        
        // create and add a camera to the scene
        let cameraNode = SCNNode()
        cameraNode.camera = SCNCamera()
        cameraNode.position = SCNVector3(x: 0, y: 0, z: 4)
        //cameraNode.rotation = SCNVector4(x: 1, y: 1, z: 1, w: 3.14)
        cameraNode.camera?.zNear = 0.01
        cameraNode.camera?.zFar = 15
        scene.rootNode.addChildNode(cameraNode)

        // add a cube
        let indices: [Int32] = [0, 1]
        let v0 = SCNVector3(0,0,0)
        let v1 = SCNVector3(1,0,0)
        let v2 = SCNVector3(0,1,0)
        let v3 = SCNVector3(0,0,1)
        
        var axisSource = SCNGeometrySource(vertices: [v0, v1])
        var axisElement = SCNGeometryElement(indices: indices, primitiveType: .line)
        var axisGeo = SCNGeometry(sources: [axisSource], elements: [axisElement])
        var axisNode = SCNNode(geometry: axisGeo)
        scene.rootNode.addChildNode(axisNode)
        
        axisSource = SCNGeometrySource(vertices: [v0, v2])
        axisElement = SCNGeometryElement(indices: indices, primitiveType: .line)
        axisGeo = SCNGeometry(sources: [axisSource], elements: [axisElement])
        axisNode = SCNNode(geometry: axisGeo)
        scene.rootNode.addChildNode(axisNode)
        
        
        axisSource = SCNGeometrySource(vertices: [v0, v3])
        axisElement = SCNGeometryElement(indices: indices, primitiveType: .line)
        axisGeo = SCNGeometry(sources: [axisSource], elements: [axisElement])
        axisNode = SCNNode(geometry: axisGeo)
        scene.rootNode.addChildNode(axisNode)
        
        let cube = SCNBox(width: 0.1, height: 0.1, length: 0.1, chamferRadius: 0)
        let node = SCNNode(geometry: cube)
        scene.rootNode.addChildNode(node)
        meshIndex = scene.rootNode.childNodes.count - 1
        
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
    
    /*
    @IBAction func meshWithIso(_ sender: Any) {
        let iso = Float(isolevel.text!)
        DispatchQueue.global().async {
            self.points = extractMesh(model: Model.sharedInstance, isolevel: iso!)
            self.isMeshed = true
            let pointCloudNode = UIFactory.mesh(from: self.points)
            let scnView = self.view as! SCNView
            if self.self.meshIndex > 0 {
                scnView.scene?.rootNode.childNodes[self.meshIndex].removeFromParentNode()
            }
            scnView.scene?.rootNode.addChildNode(pointCloudNode)
            self.meshIndex = (scnView.scene?.rootNode.childNodes.count)! - 1
        }
    }
    */
    
    /*
    @IBAction func showOnlyPoints(_ sender: Any) {
        //let iso = Int(withoutMesh.text!)
        let iso = Float(withoutMesh.text!)
        //let datasetName = "ikea-table"
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
            if self.self.meshIndex > 0 {
                scnView.scene?.rootNode.childNodes[self.meshIndex].removeFromParentNode()
            }
            scnView.scene?.rootNode.addChildNode(pointCloudNode)
            self.meshIndex = (scnView.scene?.rootNode.childNodes.count)! - 1
        }
    }
    */
    
    @IBAction func export(_ sender: Any) {
        // Export mesh at an isolovel (need to extract mesh once again, not so logical but easiest way)
        if (!isMeshed) {
            //let iso = Float(isolevel.text!)
            self.points = extractMesh(model: Model.sharedInstance, isolevel: 0.00)
            self.isMeshed = true
        }
        let path = IO.Export.toPLYFormat(mesh: points, at: "meshing.ply")!
        let tabPath = [path]
        let vc = UIActivityViewController(activityItems: tabPath, applicationActivities: [])
        present(vc, animated: true, completion: nil)
    }
    
    @IBAction func fastMeshing(_ sender: Any) {
        // On IphoneX keyboard won't close (so meshWithIso() does not work)
        // Extract mesh with isolevel = 0.0
        DispatchQueue.global().async {
            self.isMeshed = true
            
            self.points = extractMesh(model: Model.sharedInstance, isolevel: 0.00)
            let pointCloudNode = UIFactory.mesh(from: self.points)
            
            /*
            let positionSources = SCNGeometrySource(vertices: Model.sharedInstance.graph)
            let elements = SCNGeometryElement(
                data: nil,
                primitiveType: .point,
                primitiveCount: Model.sharedInstance.graph.count,
                bytesPerIndex: MemoryLayout<SCNVector3>.size
            )
            let geo = SCNGeometry(sources: [positionSources], elements: [elements])
            let pointCloudNode = SCNNode(geometry: geo)
            */
            
            let scnView = self.view as! SCNView
            if self.self.meshIndex > 0 {
                scnView.scene?.rootNode.childNodes[self.meshIndex].removeFromParentNode()
            }
            scnView.scene?.rootNode.addChildNode(pointCloudNode)
            self.meshIndex = (scnView.scene?.rootNode.childNodes.count)! - 1
        }
    }
    
    @IBAction func showWireFrame(_ sender: Any) {
        // Switch between wireframe mode and texture mode
        let scnView = self.view as! SCNView
        if wireFrame.isOn {
            scnView.scene?.rootNode.childNodes[meshIndex].geometry?.firstMaterial?.fillMode = .lines
        }
        else {
            scnView.scene?.rootNode.childNodes[meshIndex].geometry?.firstMaterial?.fillMode = .fill
        }
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Release any cached data, images, etc that aren't in use.
    }
    
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        self.isMeshed = false
        if segue.identifier == "Main" {
            if let destination = segue.destination as? ViewController {
                destination.model = Model.sharedInstance
                //destination.datasetChoice.selectedSegmentIndex  = savedDatasetIndex
                //destination.datasetSize.selectedSegmentIndex    = savedFramesIndex
            }
        }
    }
}
