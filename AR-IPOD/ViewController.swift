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

    var myVolume: Volume = Volume.sharedInstance
    var depthImage: DepthImage = DepthImage()
    var myCamera: Camera = Camera()
    var increments: Int = 1
    var timer = Double(CFAbsoluteTimeGetCurrent())
    
    @IBOutlet weak var depthView: UIImageView!
    var myDepthStrings = [String]()
    var myDepthImage: UIImage?
    var myFocus: CGFloat = 0.5
    var mySlope: CGFloat = 4.0
    var myDepthData: AVDepthData?
    var myDepthDataRaw: AVDepthData?
    var myCIImage: CIImage?
    
    @IBOutlet var isolevelLabel: UILabel!
    @IBOutlet var isoSlider: UISlider!
    @IBOutlet var datasetProgress: UIProgressView!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // Set the view's delegate
        sceneView.delegate = self
        
        // Show statistics such as fps and timing information
        sceneView.showsStatistics = true
        
        // Create a new scene
        let scene = SCNScene(named: "art.scnassets/ship.scn")!
        
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
        datasetProgress.isHidden = true
        
        // Version with dataset
        let starter = Double(CFAbsoluteTimeGetCurrent())
        let threads = [DispatchQueue(label: "thread1", qos: .userInteractive, attributes: .concurrent),
                       DispatchQueue(label: "thread2", qos: .userInteractive, attributes: .concurrent),
                       DispatchQueue(label: "thread3", qos: .userInteractive, attributes: .concurrent),
                       DispatchQueue(label: "thread4", qos: .userInteractive, attributes: .concurrent)
        ]
        let group = DispatchGroup()
        group.enter()
        threads[0].async {
            let intrinsics = importCameraIntrinsics(from: "depthIntrinsics")
            self.myCamera.update(intrinsics: intrinsics)
            group.leave()
        }
        group.enter()
        threads[1].async {
            self.myVolume.initialize()
            group.leave()
        }
        group.enter()
        threads[2].async {
            let extrinsics = importCameraPose(from: "frame-000000.pose")
            self.myCamera.update(extrinsics: extrinsics)
            group.leave()
        }
        group.enter()
        threads[3].async {
            let depthMap = importDepthMapFromTXT(from: "frame-0.depth")
            self.depthImage.update(_data: depthMap)
            group.leave()
        }
        group.wait()
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
        let extrinsics = importCameraPose(from: "frame-\(increments).pose")
        let depthMap = importDepthMapFromTXT(from: "frame-\(increments).depth")
        increments += 1
        myCamera.update(extrinsics: extrinsics)
        depthImage.update(_data: depthMap)
        myVolume.integrateDepthMap(image: depthImage, camera: &myCamera)
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
        for i in 0..<100 {
            let extrinsics = importCameraPose(from: "frame-\(i).pose")
            let depthmap = importDepthMapFromTXT(from: "frame-\(i).depth")
            self.myCamera.update(extrinsics: extrinsics)
            self.depthImage.update(_data: depthmap)
            self.myVolume.integrateDepthMap(image: self.depthImage, camera: &self.myCamera)
        }
    }
    
    @IBAction func updateIsolevel(_ sender: Any) {
        self.isolevelLabel.text = "Isolevel : \(String(self.isoSlider.value))"
    }
    @IBAction func exportVolume(_ sender: Any) {
        let points = extractMesh(volume: myVolume, isolevel: isoSlider.value)
        exportToPLY(mesh: points, at: "mesh_\(self.myVolume.size).ply")
        //exportToPLY(volume: self.myVolume, at: "volume_\(self.myVolume.size).ply")
    }
}
