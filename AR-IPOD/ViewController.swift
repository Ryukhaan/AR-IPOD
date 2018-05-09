//
//  ViewController.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 19/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import UIKit
import SceneKit
import ARKit

class ViewController: UIViewController, ARSCNViewDelegate {
    
    @IBOutlet var sceneView: ARSCNView!

    var dataset: String = "chair"
    var myVolume: Volume = Volume.sharedInstance
    var depthImage: DepthImage = DepthImage()
    var myCamera: Camera = Camera()
    var increments: Int = 1
    var timer = Double(CFAbsoluteTimeGetCurrent())
    
    @IBOutlet var volumeSize: UILabel!
    @IBOutlet var stepperSize: UIStepper!
    
    @IBOutlet weak var depthView: UIImageView!
    var myDepthStrings = [String]()
    var myDepthImage: UIImage?
    var myFocus: CGFloat = 0.5
    var mySlope: CGFloat = 4.0
    var myDepthData: AVDepthData?
    var myDepthDataRaw: AVDepthData?
    var myCIImage: CIImage?
    
    // Isolevel parameters UI
    @IBOutlet var isolevelLabel: UILabel!
    @IBOutlet var isolevelStepper: UIStepper!
    // Delta parameters UI
    @IBOutlet var deltaStepper: UIStepper!
    @IBOutlet var deltaLabel: UILabel!
    // Epsilon parameters UI
    @IBOutlet var epsilonStepper: UIStepper!
    @IBOutlet var epsilonLabel: UILabel!
    
    @IBOutlet var datasetChoice: UISegmentedControl!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // Set the view's delegate
        sceneView.delegate = self
        
        // Show statistics such as fps and timing information
        sceneView.showsStatistics = true
        
        // Create a new scene
        let scene = SCNScene()
        
        // Set the scene to the view
        sceneView.scene = scene

    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        
        // Create a session configuration
        let configuration = ARWorldTrackingConfiguration()
        //let configuration = ARFaceTrackingConfiguration()

        // Run the view's session
        sceneView.session.run(configuration)
        
        // Initialize Volume
        /*
        print("Initialize...")
        self.myVolume.initialize()
        print("Import...")
        let pointCloud = importPointCloud(fromFile: "/Users/Remi/Documents/datas/resize_result1.sdp")
         */
        
        /* SERIAL */
        /*
        let start   = CFAbsoluteTimeGetCurrent()

        let end    = CFAbsoluteTimeGetCurrent()
        let elapsedTime = Double(end) - Double(start)
        print("Time : \(elapsedTime)")
         */
        //print("Init...")
        
        /*
        myVolume.initialize()
        pointCloud = importPointCloud(fromFile: "/Users/Remi/Documents/datas/resize_result1.sdp")
        var tritri = [Vector]()
        print("Integrate...")
        //self.myVolume.falseIntegration(pointCloud: pointCloud)
        self.myVolume.cuboid(at: Point3D(2,2,2))
        print("Extract...")
        */
        
        /*
        print("Export...")
        exportToPLY(triangles: tritri, fileName: "mesh_\(self.myVolume.size).ply")
        exportToPLY(volume: self.myVolume, fileName: "volume_\(self.myVolume.size).ply")
        print("Done !")
        */
        /*
        if let frame = self.sceneView.session.currentFrame {
            self.myCamera = Camera(_intrinsics: frame.camera.intrinsics, dim: frame.camera.imageResolution)
            self.myCamera.update(position: frame.camera.transform)
        }
        else {
            assertionFailure("Camera has not been initialized ! ")
        }
        */
        
        // Version with dataset
        let starter = Double(CFAbsoluteTimeGetCurrent())
        let threads = [DispatchQueue(label: "thread1", qos: .userInteractive, attributes: .concurrent),
                       DispatchQueue(label: "thread2", qos: .userInteractive, attributes: .concurrent)
        ]
        let group = DispatchGroup()
        group.enter()
        threads[0].async {
            let intrinsics = importCameraIntrinsics(from: "depthIntrinsics", at: self.dataset)
            self.myCamera.update(intrinsics: intrinsics)
            group.leave()
        }
        group.enter()
        threads[1].async {
            self.myVolume.initialize()
            group.leave()
        }
        _ = Double(CFAbsoluteTimeGetCurrent()) - starter
        
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        
        // Pause the view's session
        sceneView.session.pause()
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Release any cached data, images, etc that aren't in use.
    }

    // MARK: - ARSCNViewDelegate
    
/*
    // Override to create and configure nodes for anchors added to the view's session.
    func renderer(_ renderer: SCNSceneRenderer, nodeFor anchor: ARAnchor) -> SCNNode? {
        let node = SCNNode()
     
        return node
    }
*/
    
    func session(_ session: ARSession, cameraDidChangeTrackingState camera: ARCamera) {
    }
    
    func session(_ session: ARSession, didFailWithError error: Error) {
        // Present an error message to the user
        
    }
    
    func sessionWasInterrupted(_ session: ARSession) {
        // Inform the user that the session has been interrupted, for example, by presenting an overlay
        
    }
    
    func sessionInterruptionEnded(_ session: ARSession) {
        // Reset tracking and/or remove existing anchors if consistent tracking is required
        
    }
    
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        // Capture DepthMap
        /*
        if frame.capturedDepthData != nil {
            myDepthData = frame.capturedDepthData?.converting(toDepthDataType: kCVPixelFormatType_DepthFloat32)
            myDepthDataRaw =  frame.capturedDepthData
            let depthDataMap = myDepthData?.depthDataMap
            CVPixelBufferLockBaseAddress(depthDataMap!, CVPixelBufferLockFlags(rawValue: 0))
            let depthPointer = unsafeBitCast(CVPixelBufferGetBaseAddress(depthDataMap!), to: UnsafeMutablePointer<Float>.self)
            /*
             * Potential Pre-processing : Median Filter.
             * We have to convert depthDataMap into an UIImage to do this.
            compCIImage(depthDataMap: depthDataMap!)
            myDepthImage = UIImage(ciImage: myCIImage!)
            depthView.image = myDepthImage
            */
            depthImage.update(_data: depthPointer)
        }
        myCamera.update(position: frame.camera.transform)
        myVolume.integrateDepthMap(image: depthImage, camera: myCamera)
         */
    }
    
    
    @IBAction func startCompute(_ sender: Any) {
        let epsilon = epsilonStepper.value * 0.05
        let delta = deltaStepper.value * 0.05
        for i in 0..<11 {
            let extrinsics = importCameraPose(from: "frame-\(i).pose", at: dataset)
            let depthmap = importDepthMapFromTXT(from: "frame-\(i).depth", at: dataset)
            self.myCamera.update(extrinsics: extrinsics)
            self.depthImage.update(_data: depthmap)
            self.myVolume.integrateDepthMap(image: self.depthImage, camera: &self.myCamera, parameters: [Float(delta), Float(epsilon)])
        }
    }
    
    @IBAction func updateEpsilon(_ sender: Any) {
        let quantity = epsilonStepper.value * 0.05
        epsilonLabel.text = "Epsilon: \(quantity)"
    }

    @IBAction func updateDelta(_ sender: Any) {
        let quantity = deltaStepper.value * 0.05
        deltaLabel.text = "Delta: \(quantity)"
    }
    
    @IBAction func increaseVolume(_ sender: Any) {
        let quantity = pow(2.0, stepperSize.value)
        self.myVolume.initialize(with: Int(quantity))
        volumeSize.text = "Volume Size : \(quantity)"
    }
    
    @IBAction func updateIsolevel(_ sender: Any) {
        let power = (isolevelStepper.value - 6)
        let newLevel = pow(10.0, power)
        if isolevelStepper.value == 0 {
            isolevelLabel.text = "Isolevel: 0"
        }
        else {
           isolevelLabel.text = "Isolevel: \(newLevel)"
        }
    }
    
    @IBAction func changeDataset(_ sender: Any) {
        switch datasetChoice.selectedSegmentIndex {
        case 0:
            dataset = "chair"
        case 1:
            dataset = "ikea-table"
        default:
            dataset = "chair"
        }
    }
    
    @IBAction func exportVolume(_ sender: Any) {
        guard let currentFrame = sceneView.session.currentFrame
            else { return }
        var isolevel: Float
        if isolevelStepper.value == 0 {
            isolevel = 0
        }
        else {
            isolevel = Float(pow(10.0, isolevelStepper.value - 6))
        }
        let points = extractMesh(volume: myVolume, isolevel: Float(isolevel))
        //sceneView.scene.rootNode.addChildNode(pointNode)
        exportToPLY(mesh: points, at: "mesh_\(dataset)_\(self.myVolume.size).ply")
        exportToPLY(volume: self.myVolume, at: "volume_\(dataset)_\(self.myVolume.size).ply")
    }
}
