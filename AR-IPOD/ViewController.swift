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
import Accelerate
import Metal

class ViewController: UIViewController, ARSCNViewDelegate, ARSessionDelegate {
    
    @IBOutlet var tx: UILabel!
    @IBOutlet var ty: UILabel!
    @IBOutlet var tz: UILabel!
    
    @IBOutlet var sceneView: ARSCNView!
    
    /*
    var passage: matrix_float4x4 = matrix_float4x4(
        float4( 0, -1,  0,  0),
        float4( 1,  0,  0,  0),
        float4( 0,  0,  1,  0),
        float4( 0,  0,  0,  1))
     */
    /*
    var right: matrix_float4x4 = matrix_float4x4(
        float4( 1,  0,  0,  0),
        float4( 0, -1,  0,  0),
        float4( 0,  0, -1,  0),
        float4( 0,  0,  0,  1))
    */
    //let systemSoundID: SystemSoundID = 1016
    var model: Model              = Model.sharedInstance
    //var myDepthImage: DepthImage    = DepthImage(onRealTime: false)
    //var myCamera: Camera            = Camera(onRealTime: false)
    
    let service:    DOFServiceManager   = DOFServiceManager()
    var deviceType: DeviceType      = .iPhoneX
    var timer                       = Double(CFAbsoluteTimeGetCurrent())
    var startIntegrating: Bool            = false
    var isSetUp: Bool               = false
    var stop: Bool                  = false
    var wait: Bool                  = false
    var timeStep: Int               = 0
    
    var acquisitionNumber: Int      = 10
    var tempMatrix: matrix_float4x4 = matrix_identity_float4x4
    //var myDepthStrings = [String]()
    //var myDepthImage: UIImage?
    //var myFocus: CGFloat = 0.5
    //var mySlope: CGFloat = 4.0
    //var myDepthData: AVDepthData?
    var myDepthDataRaw: AVDepthData?
    //var myCIImage: CIImage?
    
    //var previousLocation: SCNVector3 = SCNVector3(0, 0, 0)
    //var cameraPose: matrix_float4x4 = matrix_float4x4(diagonal: float4(1,1,1,1))
    //var oldFrame: matrix_float4x4 = matrix_float4x4(diagonal: float4(1,1,1,1))
    override func viewDidLoad() {
        
        super.viewDidLoad()
        
        // Set the view's delegate
        sceneView.delegate = self
        
        // Show statistics such as fps and timing information
        sceneView.showsStatistics = false
        
        // Create a new scene
        let scene = SCNScene()
        
        // Set the scene to the view
        sceneView.scene = scene
        service.delegate = self
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        
        // Create a session configuration
        //let configuration = ARWorldTrackingConfiguration()
        let configuration = ARFaceTrackingConfiguration()
        
        // Run the view's session
        sceneView.session.run(configuration)
        //sceneView.debugOptions = [ARSCNDebugOptions.showFeaturePoints, ARSCNDebugOptions.showWorldOrigin]
        sceneView.session.delegate = self
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
        switch camera.trackingState {
        case .notAvailable:
            sceneView.alpha = 0.2
        case .normal:
            sceneView.alpha = 0.7
        default:
            return;
        }
    }
    
    func session(_ session: ARSession, didFailWithError error: Error) {
        // Present an error message to the user
        let configuration = ARWorldTrackingConfiguration()
        configuration.planeDetection = .horizontal
        sceneView.session.run(configuration)
        sceneView.session.delegate = self
        
        sceneView.debugOptions = [ARSCNDebugOptions.showFeaturePoints, ARSCNDebugOptions.showWorldOrigin]
        //let cube = SCNBox(width: 0.1, height: 0.1, length: 0.1, chamferRadius: 0)
        //let node = SCNNode(geometry: cube)
        //sceneView.scene.rootNode.addChildNode(node)
        self.deviceType = .iPad
    }
    
    func sessionWasInterrupted(_ session: ARSession) {
        // Inform the user that the session has been interrupted, for example, by presenting an overlay
    }
    
    func sessionInterruptionEnded(_ session: ARSession) {
        // Reset tracking and/or remove existing anchors if consistent tracking is required
    }
    
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        // Capture DepthMap
        var Rt = frame.camera.transform
        switch self.deviceType {
        case .iPad:
            //NSLog("%@", "\(cameraPose)")
            self.ty.numberOfLines = 1
            self.tz.numberOfLines = 4
            let f = ".2"
            self.ty.text = """
            \((Rt.columns.3.x.format(f)))\t\(Rt.columns.3.y.format(f))\t\(Rt.columns.3.z.format(f))
            """
            /*
             self.ty.text = """
             \(Rt.columns.0.x.format(f))\t \(Rt.columns.1.x.format(f))\t \(Rt.columns.2.x.format(f))\t \(Rt.columns.3.x.format(f))
             \(Rt.columns.0.y.format(f))\t \(Rt.columns.1.y.format(f))\t \(Rt.columns.2.y.format(f))\t \(Rt.columns.3.y.format(f))
             \(Rt.columns.0.z.format(f))\t \(Rt.columns.1.z.format(f))\t \(Rt.columns.2.z.format(f))\t \(Rt.columns.3.z.format(f))
             \(Rt.columns.0.w.format(f))\t \(Rt.columns.1.w.format(f))\t \(Rt.columns.2.w.format(f))\t \(Rt.columns.3.w.format(f))
             """
             
             self.tz.text = """
             \(M.columns.0.x.format(f))\t \(M.columns.1.x.format(f))\t \(M.columns.2.x.format(f))\t \(M.columns.3.x.format(f))
             \(M.columns.0.y.format(f))\t \(M.columns.1.y.format(f))\t \(M.columns.2.y.format(f))\t \(M.columns.3.y.format(f))
             \(M.columns.0.z.format(f))\t \(M.columns.1.z.format(f))\t \(M.columns.2.z.format(f))\t \(M.columns.3.z.format(f))
             \(M.columns.0.w.format(f))\t \(M.columns.1.w.format(f))\t \(M.columns.2.w.format(f))\t \(M.columns.3.w.format(f))
             """
             */
            //service.send(transform: Rt)
            self.tempMatrix = Rt
        case .iPhoneX:
            let t = float4(Model.sharedInstance.camera.translation, 1)
            Rt.columns.3 = t
            //Rt = simd_transpose(passage) * Rt * passage
            //Rt = right * Rt
            //Rt.columns.3 = float4(t.x, t.y, t.z, 1)
            //M.columns.3 = t
            self.ty.numberOfLines = 4
            let f = ".2"
            
             self.ty.text = """
             \(Rt.columns.0.x.format(f)) \(Rt.columns.1.x.format(f)) \(Rt.columns.2.x.format(f)) \(Rt.columns.3.x.format(f))
             \(Rt.columns.0.y.format(f)) \(Rt.columns.1.y.format(f)) \(Rt.columns.2.y.format(f)) \(Rt.columns.3.y.format(f))
             \(Rt.columns.0.z.format(f)) \(Rt.columns.1.z.format(f)) \(Rt.columns.2.z.format(f)) \(Rt.columns.3.z.format(f))
             \(Rt.columns.0.w.format(f)) \(Rt.columns.1.w.format(f)) \(Rt.columns.2.w.format(f)) \(Rt.columns.3.w.format(f))
             """
            /*
            self.ty.text = """
            \((M.columns.3.x.format(f)))\t\(M.columns.3.y.format(f))\t\(M.columns.3.z.format(f))
            """
            */
            if let image = frame.capturedDepthData
            {
                self.myDepthDataRaw =  image
                let depthDataMap = image.depthDataMap
                if startIntegrating {
                    CVPixelBufferLockBaseAddress(depthDataMap, CVPixelBufferLockFlags(rawValue: 0))
                    let depthPointer = unsafeBitCast(CVPixelBufferGetBaseAddress(depthDataMap), to: UnsafeMutablePointer<Float>.self)
                    
                    Model.sharedInstance.image.width    = Int(CVPixelBufferGetWidth(depthDataMap))
                    Model.sharedInstance.image.height   = Int(CVPixelBufferGetHeight(depthDataMap))
                    Model.sharedInstance.camera.width   = Int(CVPixelBufferGetWidth(depthDataMap))
                    Model.sharedInstance.camera.height  = Int(CVPixelBufferGetHeight(depthDataMap))
                    if let calibration = image.cameraCalibrationData {
                        let frameReference = calibration.intrinsicMatrixReferenceDimensions
                        Model.sharedInstance.parameters["cy"] = Float(frameReference.width / CGFloat(Model.sharedInstance.image.width))
                        Model.sharedInstance.parameters["cx"] = Float(frameReference.height / CGFloat(Model.sharedInstance.image.height))
                        Model.sharedInstance.update(intrinsics: calibration.intrinsicMatrix)
                    }
                    
                    //Model.sharedInstance.push(data: depthPointer)
                    //Model.sharedInstance.computeDepthsMedian()
                    Model.sharedInstance.update(data: depthPointer)
                    
                    var depthmap = Model.sharedInstance.image.data
                    bridge_median_filter(&depthmap,
                                         2,
                                         Int32(Model.sharedInstance.camera.width),
                                         Int32(Model.sharedInstance.camera.height))
                    Model.sharedInstance.update(data: depthmap)
                    
                    self.startIntegrating = false
                    Model.sharedInstance.update(translation: Rt)
                    Model.sharedInstance.update(rotation: Rt)
                    
                    let group = DispatchGroup()
                    group.enter()
                    DispatchQueue.global().async {
                        /*
                        for i in 0..<Model.sharedInstance.image.height {
                            for j in 0..<Model.sharedInstance.image.width {
                                let depth = Model.sharedInstance.image.at(row: i, column: j)
                                let cx: Float = 6
                                let cy: Float = 6
                                let homogene = float4(Float(j) * depth * cx, Float(i) * depth * cy, depth, 1)
                                let K = simd_inverse(Model.sharedInstance.camera.intrinsics)
                                //let Kmod = simd_transpose(self.passage) * K * self.passage
                                let local = simd_mul(K, homogene)
                                let rlocal = self.right * local
                                let global = simd_mul(Rt, rlocal)
                                let rglobal = self.passage * global
                                Model.sharedInstance.graph.append(SCNVector3(rglobal.x, rglobal.y, rglobal.z))
                                
                            }
                        }
                        */
                        Model.sharedInstance.integrate()
                        group.leave()
                    }
                    group.wait()
                    self.service.send(alert: Constant.Code.Integration.hasFinished)
                }
            }
        }
    }
    
    func renderer(_ renderer: SCNSceneRenderer, willRenderScene scene: SCNScene, atTime time: TimeInterval) {
        // Pass
    }
    
    @IBAction func resetModel(_ sender: Any) {
        setUpBasis()
        self.tx.text = "Reinitialized"
        switch deviceType {
        case .iPad:
            service.send(alert: Constant.Code.Integration.reset)
        case .iPhoneX:
            Model.sharedInstance.reinit()
            //Model.sharedInstance.graph = [SCNVector3](repeating: SCNVector3(0,0,0), count: 0)
        }
    }
    
    
    @IBAction func startRealTimeIntegration(_ sender: Any) {
        //guard let Rt = sceneView.session.currentFrame else { return }
        self.stop = !self.stop
        switch deviceType {
        case .iPad:
            if (!self.stop) {
                self.tx.text = "Integrating..."
                service.send(transform: self.tempMatrix)
                service.send(alert: Constant.Code.Integration.isStarting)
            }
        case .iPhoneX:
            self.startIntegrating = true
        }
    }
    
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        if segue.identifier == "ScenePoints" {
            if let destination = segue.destination as? SceneViewController {
                destination.model = Model.sharedInstance
            }
        }
    }
    
    func setUpBasis() {
        timeStep = 0
        var N = matrix_identity_float4x4
        switch deviceType {
        case .iPad:
            N = matrix_float4x4(
                float4( 0, -1,  0,  0),
                float4(-1,  0,  0,  0),
                float4( 0,  0,  1,  0),
                float4( 0,  0,  0,  1))
            //return;
        case .iPhoneX:
            N = matrix_float4x4(
                float4( 0, -1,  0,  0),
                float4(-1,  0,  0,  0),
                float4( 0,  0, -1,  0),
                float4( 0,  0,  0,  1))
        }
        if (!self.isSetUp) {
            self.sceneView.session.setWorldOrigin(relativeTransform: N)
            self.isSetUp = true
        }
    }
    
}

extension ViewController : DOFServiceManagerDelegate {
    
    func connectedDevicesChanged(manager : DOFServiceManager, connectedDevices: [String]) {
        print("Connections: \(connectedDevices)")
        if (connectedDevices.count > 0) {
            self.tx.text = "Connected"
        }
        else {
            self.tx.text = "Device Not Found"
        }
    }
    
    func transformChanged(manager : DOFServiceManager, transform: matrix_float4x4) {
        // Update Rotation and Transformation : Camera Position
        OperationQueue.main.addOperation {
            switch self.deviceType {
            case .iPhoneX:
                Model.sharedInstance.update(translation: transform)
                self.tx.text = "Received"
            case .iPad:
                return;
            }
        }
    }
    
    func integrationFinished(manager: DOFServiceManager) {
        // IPAD : Receive message when integration has been finished
        //guard let Rt = self.sceneView.session.currentFrame else { return }
        OperationQueue.main.addOperation {
            switch self.deviceType {
            case .iPad:
                self.timeStep += 1
                self.tx.text = "Finished : Time \(self.timeStep)"
                let halt = (self.timeStep % self.acquisitionNumber == 0)
                if (halt) {
                    self.stop = true
                }
                if (!self.stop) {
                    self.service.send(transform: self.tempMatrix)
                    self.service.send(alert: Constant.Code.Integration.isStarting)
                }
            case .iPhoneX:
                self.startIntegrating = false
            }
        }
    }
    
    func startIntegrating(manager: DOFServiceManager) {
        // IPHONE : Receive message to start integrating
        OperationQueue.main.addOperation {
            switch self.deviceType {
            case .iPhoneX:
                self.startIntegrating = true
            default:
                self.tx.text = "Integrating..."
            }
        }
    }
    
    func resetModel(manager: DOFServiceManager) {
        // IPHONE : Resetting model
        setUpBasis()
        OperationQueue.main.addOperation {
            switch self.deviceType {
            case .iPhoneX:
                Model.sharedInstance.reinit()
                self.tx.text = "Reinitialized"
            default:
                self.tx.text = "Reinitialized"
            }
        }
    }
}
