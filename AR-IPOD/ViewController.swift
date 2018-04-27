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
    
    @IBOutlet weak var depthView: UIImageView!
    var myDepthStrings = [String]()
    var myDepthImage: UIImage?
    var myFocus: CGFloat = 0.5
    var mySlope: CGFloat = 4.0
    var myDepthData: AVDepthData?
    var myDepthDataRaw: AVDepthData?
    var myCIImage: CIImage?
    
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
        //let configuration = ARWorldTrackingConfiguration()
        let configuration = ARFaceTrackingConfiguration()

        // Run the view's session
        sceneView.session.run(configuration)
        
        // Initialize Volume
        /*
        print("Initialize...")
        self.myVolume.initialize()
        print("Import...")
        let pointCloud = importPointCloud(fromFile: "/Users/Remi/Documents/datas/resize_result1.sdp")
         */
        let start   = CFAbsoluteTimeGetCurrent()
        
        let thread1 = DispatchQueue(label: "Thread1", attributes: .concurrent)
        let thread2 = DispatchQueue(label: "Thread2", attributes: .concurrent)
        let group = DispatchGroup()
        var pointCloud = PointCloud()
        /* SERIAL */
        
        print("Initialize...")
        group.enter()
        thread1.async {
            self.myVolume.initialize()
            group.leave()
        }
        group.enter()
        thread2.async {
            pointCloud = importPointCloud(fromFile: "/Users/Remi/Documents/datas/resize_result1.sdp")
            group.leave()
        }
        group.wait()
        group.notify(queue: .main) {
            let end    = CFAbsoluteTimeGetCurrent()
            let elapsedTime = Double(end) - Double(start)
            print("Time : \(elapsedTime)")
            print("Integrate...\(self.myVolume.centroids.count)")
            /*
             print("Import...")
             let pointCloud = importPointCloud(fromFile: "/Users/Remi/Documents/datas/resize_result1.sdp")
             myVolume.falseIntegration(pointCloud: pointCloud)
             print("Convert...")
             let cells = convertVolumeIntoCells(volume: myVolume)
             print("Extract...")
             let tritri = extractMesh(from: cells, isolevel: 20)
             print("Export...")
             exportToPLY(triangles: tritri, fileName: "mesh_\(myVolume.size).ply")
             exportToPLY(volume: myVolume, fileName: "volume_\(myVolume.size).ply")
             print("Done !")
             */
            if let frame = self.sceneView.session.currentFrame {
                self.myCamera = Camera(_intrinsics: frame.camera.intrinsics, dim: frame.camera.imageResolution)
                self.myCamera.update(position: frame.camera.transform)
                self.myVolume.initialize()
            }
            else {
                assertionFailure("Camera has not been initialized ! ")
            }
        }
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
    }
}
