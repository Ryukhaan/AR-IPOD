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
    
    let systemSoundID: SystemSoundID = 1016
    var myModel: Model              = Model.sharedInstance
    //var myDepthImage: DepthImage    = DepthImage(onRealTime: false)
    //var myCamera: Camera            = Camera(onRealTime: false)
    
    let service:    DOFServiceManager   = DOFServiceManager()
    var deviceType: DeviceType      = .Iphone
    let batchSize:  Int             = 3
    var sizeOfDataset: Int          = 1
    var nameOfDataset: String       = "tasse-set"
    var numberOfIterations: Int     = 0
    var timer                       = Double(CFAbsoluteTimeGetCurrent())
    var inRealTime: Bool            = false
    
    @IBOutlet var datasetSizeField: UITextField!
    @IBOutlet var integrationProgress: UIProgressView!
    
    @IBOutlet weak var depthView: UIImageView!
    var myDepthStrings = [String]()
    //var myDepthImage: UIImage?
    var myFocus: CGFloat = 0.5
    var mySlope: CGFloat = 4.0
    var myDepthData: AVDepthData?
    var myDepthDataRaw: AVDepthData?
    var myCIImage: CIImage?
    
    @IBOutlet var datasetChoice: UISegmentedControl!
    @IBOutlet var datasetSize: UISegmentedControl!
    
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
        //guard let currentFrame = sceneView.session.currentFrame
        //    else { return }
    }
    
    func session(_ session: ARSession, didFailWithError error: Error) {
        // Present an error message to the user
        let configuration = ARWorldTrackingConfiguration()
        sceneView.session.run(configuration)
        sceneView.session.delegate = self
        
        deviceType = .IPad
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
         if inRealTime
         {
         //let camera = frame.camera.trackingState
         //Model.sharedInstance.update(rotation: frame.camera.transform)
         Model.sharedInstance.update(intrinsics: frame.camera.intrinsics)
         if frame.capturedDepthData != nil
         {
         self.myDepthData = frame.capturedDepthData?.converting(toDepthDataType: kCVPixelFormatType_DepthFloat32)
         self.myDepthDataRaw =  frame.capturedDepthData
         let depthDataMap = self.myDepthData?.depthDataMap
         CVPixelBufferLockBaseAddress(depthDataMap!, CVPixelBufferLockFlags(rawValue: 0))
         let depthPointer = unsafeBitCast(CVPixelBufferGetBaseAddress(depthDataMap!), to: UnsafeMutablePointer<Float>.self)
         Model.sharedInstance.image.width = Int(CVPixelBufferGetWidth(depthDataMap!))
         Model.sharedInstance.image.height = Int(CVPixelBufferGetHeight(depthDataMap!))
         Model.sharedInstance.camera.width = Int(CVPixelBufferGetWidth(depthDataMap!))
         Model.sharedInstance.camera.height = Int(CVPixelBufferGetHeight(depthDataMap!))
         //let frameReference = self.myDepthDataRaw!.cameraCalibrationData!.intrinsicMatrixReferenceDimensions
         // We have to convert depthDataMap into an UIImage to perform some pre-processing like filtering.
         //compCIImage(depthDataMap: depthDataMap!)
         //myDepthImage = UIImage(ciImage: myCIImage!)
         //depthView.image = myDepthImage
         //self.myDepthImage.update(_data: depthPointer)
         //self.myDepthImage.update(_data: depthPointer)
         //self.myCamera.update(extrinsics: frame.camera.transform)
         let last_points = Model.sharedInstance.image.data
         //Model.sharedInstance.image.push(map: depthPointer)
         Model.sharedInstance.update(data: depthPointer)
         //Model.sharedInstance.createMedianDepthMap()
         let current_points = Model.sharedInstance.image.data
         Model.sharedInstance.globalRegistration(previous: last_points, current: current_points)
         self.numberOfIterations += 1
         }
         if self.numberOfIterations >= 6
         {
         Model.sharedInstance.integrate()
         self.numberOfIterations = 0
         }
         }
         */
        let cameraPose = frame.camera.transform
        switch self.deviceType {
        case .IPad:
            //NSLog("%@", "\(cameraPose)")
            service.send(transform: cameraPose)
        case .Iphone:
            if inRealTime {
                Model.sharedInstance.update(intrinsics: frame.camera.intrinsics)
                if frame.capturedDepthData != nil
                {
                    self.myDepthData = frame.capturedDepthData?.converting(toDepthDataType: kCVPixelFormatType_DepthFloat32)
                    self.myDepthDataRaw =  frame.capturedDepthData
                    let depthDataMap = self.myDepthData?.depthDataMap
                    CVPixelBufferLockBaseAddress(depthDataMap!, CVPixelBufferLockFlags(rawValue: 0))
                    let depthPointer = unsafeBitCast(CVPixelBufferGetBaseAddress(depthDataMap!), to: UnsafeMutablePointer<Float>.self)
                    Model.sharedInstance.image.width = Int(CVPixelBufferGetWidth(depthDataMap!))
                    Model.sharedInstance.image.height = Int(CVPixelBufferGetHeight(depthDataMap!))
                    Model.sharedInstance.camera.width = Int(CVPixelBufferGetWidth(depthDataMap!))
                    Model.sharedInstance.camera.height = Int(CVPixelBufferGetHeight(depthDataMap!))
                    let frameReference = self.myDepthDataRaw!.cameraCalibrationData!.intrinsicMatrixReferenceDimensions
                    Model.sharedInstance.parameters["cx"] = Float(frameReference.width / CGFloat(Model.sharedInstance.image.width))
                    Model.sharedInstance.parameters["cy"] = Float(frameReference.height / CGFloat(Model.sharedInstance.image.height))
                    // We have to convert depthDataMap into an UIImage to perform some pre-processing like filtering.
                    //compCIImage(depthDataMap: depthDataMap!)
                    //myDepthImage = UIImage(ciImage: myCIImage!)
                    //depthView.image = myDepthImage
                    //self.myDepthImage.update(_data: depthPointer)
                    //self.myDepthImage.update(_data: depthPointer)
                    //self.myCamera.update(extrinsics: frame.camera.transform)
                    //let last_points = Model.sharedInstance.image.data
                    //Model.sharedInstance.image.push(map: depthPointer)
                    Model.sharedInstance.update(data: depthPointer)
                    var depthmap = Model.sharedInstance.image.data
                    bridge_median_filter(&depthmap,
                                         2,
                                         Int32(Model.sharedInstance.camera.width),
                                         Int32(Model.sharedInstance.camera.height))
                    Model.sharedInstance.update(data: depthmap)
                    self.inRealTime = false
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
    
    
    @IBAction func resetModel(_ sender: Any) {
        if self.deviceType == .IPad {
            service.send(alert: Constant.Code.Integration.reset)
        }
        /*
        //let group = DispatchGroup()
        integrationProgress.progress = 0.0
        integrationProgress.isHidden = false
        //inRealTime = false
        //self.myModel.switchTo(realTime: inRealTime)
        Model.sharedInstance.reinit()
        
        timer = Double(CFAbsoluteTimeGetCurrent())
        var depthmap: [Float]
        for i in 0..<self.sizeOfDataset {
            //DispatchQueue.main.asyncAfter(deadline: .now() + Double(3*i)) {
            /*
             extrinsics = Import.cameraPose(
             from: "frame-\(i).pose",
             at: self.nameOfDataset,
             type: Model.sharedInstance.type)
             */
            depthmap = Import.depthMapFromTXT(
                from: "frame-\(i).depth",
                at: self.nameOfDataset,
                type: Model.sharedInstance.type)
            bridge_median_filter(&depthmap,
                                 2,
                                 Int32(Model.sharedInstance.camera.width),
                                 Int32(Model.sharedInstance.camera.height))
            //Model.sharedInstance.update(rotation: extrinsics)
            //Model.sharedInstance.update(translation: extrinsics)
            let last_points = Model.sharedInstance.image.data
            Model.sharedInstance.update(data: depthmap)
            if i > 0 {
                let current_points = Model.sharedInstance.image.data
                Model.sharedInstance.globalRegistration(previous: last_points, current: current_points)
            }
            Model.sharedInstance.integrate()
        }
        let end = Double(CFAbsoluteTimeGetCurrent()) - timer
        self.displayAlertMessage(title: "Fin Acquisition", message: "\(end)", handler: {_ in
            self.integrationProgress.isHidden = true
            self.integrationProgress.progress = 0.0
        })
        */
    }
    
    @IBAction func changeDataset(_ sender: Any) {
        switch datasetChoice.selectedSegmentIndex {
        case 0:
            nameOfDataset = "cube-set"
            Model.sharedInstance.switchTo(type: .Iphone)
        //myModel = Model(from: Model.sharedInstance, to: .Iphone)
        case 1:
            nameOfDataset = "ikea-table"
            //myModel = Model(from: Model.sharedInstance, to: .Kinect)
            Model.sharedInstance.switchTo(type: .Kinect)
        default: break
        }
        let intrinsic = Import.intrinsics(from: "depthIntrinsics", at: nameOfDataset, type: Model.sharedInstance.type)
        Model.sharedInstance.update(intrinsics: intrinsic)
    }
    
    @IBAction func updateDatasetSize(_ sender: Any) {
        let size = Int(datasetSizeField.text!)
        if let newSize = size {
            sizeOfDataset = newSize
            tx.text = datasetSizeField.text
        }
    }
    
    @IBAction func changeDatasetSize(_ sender: Any) {
        switch datasetSize.selectedSegmentIndex {
        case 0:
            sizeOfDataset = 10
        case 1:
            sizeOfDataset = 100
        case 2:
            sizeOfDataset = 200
        default:
            sizeOfDataset = 10
        }
    }
    
    @IBAction func startRealTimeIntegration(_ sender: Any) {
        //let cameraPose = sceneView.session.currentFrame?.camera.transform
        //let frameRef = myDepthDataRaw!.cameraCalibrationData!.intrinsicMatrixReferenceDimensions
        if deviceType == .IPad {
            self.tx.text = "Waiting..."
            //service.send(transform: cameraPose!)
            //service.send(alert: Constant.Code.Integration.cxcy, cxUpdated: <#T##String#>, cyUpdated: <#T##String#>)
            service.send(alert: Constant.Code.Integration.isStarting)
        }
    }
    
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        if segue.identifier == "ScenePoints" {
            if let destination = segue.destination as? SceneViewController {
                destination.myModel = Model.sharedInstance
                destination.savedDatasetIndex       = datasetChoice.selectedSegmentIndex
                destination.savedFramesIndex        = datasetSize.selectedSegmentIndex
            }
        }
    }
    
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
}

extension ViewController : DOFServiceManagerDelegate {
    
    func connectedDevicesChanged(manager : DOFServiceManager, connectedDevices: [String]) {
        print("Connections: \(connectedDevices)")
    }
    
    func transformChanged(manager : DOFServiceManager, transform: matrix_float4x4) {
        let M = transform
        NSLog("%@", "\(M)")
        OperationQueue.main.addOperation {
            if (self.deviceType == .Iphone) {
                Model.sharedInstance.update(rotation: transform)
                Model.sharedInstance.update(translation: transform)
                self.tx.text = "Received"
            }
        }
    }
    
    func integrationFinished(manager: DOFServiceManager) {
        OperationQueue.main.addOperation {
            switch self.deviceType {
            case .IPad:
                self.tx.text = "Finished"
            case .Iphone:
                self.inRealTime = false
            }
        }
    }
    
    func startIntegrating(manager: DOFServiceManager) {
        OperationQueue.main.addOperation {
            switch self.deviceType {
            case .Iphone:
                self.inRealTime = true
            default:
                return;
            }
        }
    }
    
    func resetModel(manager: DOFServiceManager) {
        OperationQueue.main.addOperation {
            switch self.deviceType {
            case .Iphone:
                Model.sharedInstance.reinit()
            default:
                return;
            }
        }
    }
    
    func updateCxCy(manager: DOFServiceManager, points: CGSize) {
        OperationQueue.main.addOperation {
            switch self.deviceType {
            case .Iphone:
                Model.sharedInstance.parameters["cx"] = Float(points.width)
                Model.sharedInstance.parameters["cy"] = Float(points.height)
            default:
                return;
            }
        }
    }
 }
