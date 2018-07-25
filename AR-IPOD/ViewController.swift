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

class ViewController: UIViewController, ARSCNViewDelegate, ARSessionDelegate {
    
    @IBOutlet var tx: UILabel!
    @IBOutlet var ty: UILabel!
    @IBOutlet var tz: UILabel!
    
    @IBOutlet var sceneView: ARSCNView!
    
    var passage: matrix_float4x4 = matrix_float4x4(
        float4( 0, -1,  0,  0),
        float4( 1,  0,  0,  0),
        float4( 0,  0, -1,  0),
        float4( 0,  0,  0,  1))
    
    let systemSoundID: SystemSoundID = 1016
    var myModel: Model              = Model.sharedInstance
    //var myDepthImage: DepthImage    = DepthImage(onRealTime: false)
    //var myCamera: Camera            = Camera(onRealTime: false)
    
    let service:    DOFServiceManager   = DOFServiceManager()
    var deviceType: DeviceType      = .iPhoneX
    let batchSize:  Int             = 3
    var sizeOfDataset: Int          = 1
    var nameOfDataset: String       = "tasse-set"
    var numberOfIterations: Int     = 0
    var timer                       = Double(CFAbsoluteTimeGetCurrent())
    var inRealTime: Bool            = false
    var isSetUp: Bool               = false
    
    var myDepthStrings = [String]()
    //var myDepthImage: UIImage?
    var myFocus: CGFloat = 0.5
    var mySlope: CGFloat = 4.0
    var myDepthData: AVDepthData?
    var myDepthDataRaw: AVDepthData?
    var myCIImage: CIImage?
    
    var previousLocation: SCNVector3 = SCNVector3(0, 0, 0)
    var cameraPose: matrix_float4x4 = matrix_float4x4(diagonal: float4(1,1,1,1))
    var oldFrame: matrix_float4x4 = matrix_float4x4(diagonal: float4(1,1,1,1))
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
        service.delegate = self
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        
        // Create a session configuration
        //let configuration = ARWorldTrackingConfiguration()
        let configuration = ARFaceTrackingConfiguration()
        
        // Run the view's session
        sceneView.session.run(configuration)
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
        let Rt = frame.camera.transform
        var M = frame.camera.transform
        let t = float4(Model.sharedInstance.camera.translation, 1)
        M.columns.3 = t
        M = simd_transpose(passage) * M * passage
        switch self.deviceType {
        case .iPad:
            //NSLog("%@", "\(cameraPose)")
            self.ty.numberOfLines = 4
            self.tz.numberOfLines = 4
            //let f = ".2"
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
            service.send(transform: Rt)
            //previousLocation = location
        case .iPhoneX:
            self.ty.numberOfLines = 4
            //let f = ".2"
            /*
            self.ty.text = """
            \(M.columns.0.x.format(f)) \(M.columns.1.x.format(f)) \(M.columns.2.x.format(f)) \(M.columns.3.x.format(f))
            \(M.columns.0.y.format(f)) \(M.columns.1.y.format(f)) \(M.columns.2.y.format(f)) \(M.columns.3.y.format(f))
            \(M.columns.0.z.format(f)) \(M.columns.1.z.format(f)) \(M.columns.2.z.format(f)) \(M.columns.3.z.format(f))
            \(M.columns.0.w.format(f)) \(M.columns.1.w.format(f)) \(M.columns.2.w.format(f)) \(M.columns.3.w.format(f))
            """
            */
            if let image = frame.capturedDepthData
            {
                self.myDepthDataRaw =  image
                let depthDataMap = image.depthDataMap
                CVPixelBufferLockBaseAddress(depthDataMap, CVPixelBufferLockFlags(rawValue: 0))
                let depthPointer = unsafeBitCast(CVPixelBufferGetBaseAddress(depthDataMap), to: UnsafeMutablePointer<Float>.self)
                
                Model.sharedInstance.image.width    = Int(CVPixelBufferGetWidth(depthDataMap))
                Model.sharedInstance.image.height   = Int(CVPixelBufferGetHeight(depthDataMap))
                Model.sharedInstance.camera.width   = Int(CVPixelBufferGetWidth(depthDataMap))
                Model.sharedInstance.camera.height  = Int(CVPixelBufferGetHeight(depthDataMap))
                let frameReference = image.cameraCalibrationData!.intrinsicMatrixReferenceDimensions
                Model.sharedInstance.parameters["cy"] = Float(frameReference.width / CGFloat(Model.sharedInstance.image.width))
                Model.sharedInstance.parameters["cx"] = Float(frameReference.height / CGFloat(Model.sharedInstance.image.height))
                
                Model.sharedInstance.update(intrinsics: image.cameraCalibrationData!.intrinsicMatrix)
                Model.sharedInstance.update(translation: M)
                Model.sharedInstance.update(rotation: M)
                Model.sharedInstance.push(data: depthPointer)
                
                if inRealTime {
                    
                    self.inRealTime = false
                    Model.sharedInstance.computeDepthsMedian()
                    //Model.sharedInstance.update(data: depthPointer)
                    
                    var depthmap = Model.sharedInstance.image.data
                    bridge_median_filter(&depthmap,
                                         2,
                                         Int32(Model.sharedInstance.camera.width),
                                         Int32(Model.sharedInstance.camera.height))
                    Model.sharedInstance.update(data: depthmap)
                    
                    DispatchQueue.global().async {
                        Model.sharedInstance.integrate()
                        DispatchQueue.main.async {
                            self.service.send(alert: Constant.Code.Integration.hasFinished)
                        }
                    }
                    
                }
            }
        }
    }
    
    func renderer(_ renderer: SCNSceneRenderer, willRenderScene scene: SCNScene, atTime time: TimeInterval) {
        // Pass
    }
    
    @IBAction func resetModel(_ sender: Any) {
        setUpBasis()
        switch deviceType {
        case .iPad:
            service.send(alert: Constant.Code.Integration.reset)
        case .iPhoneX:
            Model.sharedInstance.reinit()
        }
    }

    
    @IBAction func startRealTimeIntegration(_ sender: Any) {
        switch deviceType {
        case .iPad:
            self.tx.text = "Integrating..."
            service.send(alert: Constant.Code.Integration.isStarting)
        case .iPhoneX:
            self.inRealTime = true
        }
    }
    
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        if segue.identifier == "ScenePoints" {
            if let destination = segue.destination as? SceneViewController {
                destination.myModel = Model.sharedInstance
            }
        }
    }
    /*
    func displayAlertMessage(title: String, message: String, handler: @escaping ((UIAlertAction) -> Void)) {
        do {
            try AVAudioSession.sharedInstance().setCategory(AVAudioSessionCategoryPlayback)
            try AVAudioSession.sharedInstance().setActive(true)
            let player = try AVAudioPlayer(contentsOf: URL(fileURLWithPath: Sounds.beep!), fileTypeHint: AVFileType.wav.rawValue)
            player.prepareToPlay()
            player.play()
            sleep(1)
            let alert = UIAlertController(title: title, message: message, preferredStyle: .alert)
            alert.addAction(UIAlertAction(title: "OK", style: .default, handler: handler))
            self.present(alert, animated: true, completion: nil)
        }
        catch {}
    }
    */
    
    func setUpBasis() {
        var N = matrix_identity_float4x4
        switch deviceType {
        case .iPad:
            N = matrix_float4x4(
                float4( 0, -1,  0,  0),
                float4(-1,  0,  0,  0),
                float4( 0,  0,  1,  0),
                float4( 0,  0,  0,  1))
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
    }
    
    func transformChanged(manager : DOFServiceManager, transform: matrix_float4x4) {
        // Update Rotation and Transformation : Camera Position
        OperationQueue.main.addOperation {
            switch self.deviceType {
            case .iPhoneX:
                //print(transform.columns.3)
                Model.sharedInstance.update(translation: transform)
                //print(Model.sharedInstance.camera.translation)
                self.tx.text = "Received"
            case .iPad:
                return;
            }
        }
    }
    
    func integrationFinished(manager: DOFServiceManager) {
        // IPAD : Receive message when integration has been finished
        OperationQueue.main.addOperation {
            switch self.deviceType {
            case .iPad:
                self.tx.text = "Finished"
                self.service.send(alert: Constant.Code.Integration.isStarting)
            case .iPhoneX:
                self.inRealTime = false
            }
        }
    }
    
    func startIntegrating(manager: DOFServiceManager) {
        // IPHONE : Receive message to start integrating
        OperationQueue.main.addOperation {
            switch self.deviceType {
            case .iPhoneX:
                self.inRealTime = true
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
