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
    
    let batchSize:  Int             = 3
    var sizeOfDataset: Int          = 1
    var nameOfDataset: String       = "chair"
    var numberOfIterations: Int     = 0
    var timer                       = Double(CFAbsoluteTimeGetCurrent())
    var inRealTime: Bool            = false
    
    @IBOutlet var datasetSizeField: UITextField!
    @IBOutlet var volumeSize: UILabel!
    @IBOutlet var stepperSize: UIStepper!
    @IBOutlet var integrationProgress: UIProgressView!
    
    @IBOutlet weak var depthView: UIImageView!
    var myDepthStrings = [String]()
    //var myDepthImage: UIImage?
    var myFocus: CGFloat = 0.5
    var mySlope: CGFloat = 4.0
    var myDepthData: AVDepthData?
    var myDepthDataRaw: AVDepthData?
    var myCIImage: CIImage?
    
    // Isolevel parameters UI
    @IBOutlet var dimensionLabel: UILabel!
    @IBOutlet var dimensionStepper: UIStepper!
    let lambdaTick = 1.0
    // Delta parameters UI
    @IBOutlet var deltaStepper: UIStepper!
    @IBOutlet var deltaLabel: UILabel!
    let deltaTick = 0.05
    // Epsilon parameters UI
    @IBOutlet var epsilonStepper: UIStepper!
    @IBOutlet var epsilonLabel: UILabel!
    let epsilonTick = 0.02
    
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
        
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        
        // Create a session configuration
        //let configuration = ARWorldTrackingConfiguration()
        let configuration = ARFaceTrackingConfiguration()
        
        // Run the view's session
        sceneView.session.run(configuration)
        sceneView.session.delegate = self
        //sceneView.debugOptions = [ARSCNDebugOptions.showWorldOrigin, ARSCNDebugOptions.showFeaturePoints]
        //sceneView.session.setWorldOrigin(relativeTransform: matrix_float4x4(diagonal: [1,1,1,1]))
        
        // Version with dataset
        let starter = Double(CFAbsoluteTimeGetCurrent())
        let threads = [DispatchQueue(label: "thread1", qos: .userInteractive, attributes: .concurrent),
                       DispatchQueue(label: "thread2", qos: .userInteractive, attributes: .concurrent)
        ]
        let group = DispatchGroup()
        group.enter()
        threads[0].async {
            let intrinsics = importCameraIntrinsics(from: "depthIntrinsics", at: self.nameOfDataset)
            self.myModel.update(intrinsics: intrinsics)
            group.leave()
        }
        group.enter()
        threads[1].async {
            self.myModel.reallocateVoxels()
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
        //guard let currentFrame = sceneView.session.currentFrame
        //    else { return }
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
        integrationProgress.progress = 0.0
        integrationProgress.isHidden = false
        //var k = 0
        if inRealTime
        {
            //tx.text = "\(frame.camera.intrinsics.columns.2.x)"//"\(self.myCamera.extrinsics.columns.3.x)"
            //ty.text = "\(frame.camera.intrinsics.columns.2.y)"//"\(self.myCamera.extrinsics.columns.3.y)"
            //tz.text = "\(frame.camera.intrinsics.columns.2.z)"//"\(self.myCamera.extrinsics.columns.3.z)"
            //let camera = frame.camera.trackingState
            self.myModel.update(extrinsics: frame.camera.transform, onlyRotation: true)
            self.myModel.update(intrinsics: frame.camera.intrinsics)
            /*
             if let points = frame.rawFeaturePoints {
             self.myVolume.integrate(points: points, camera: self.myCamera)
             }
             */
            if frame.capturedDepthData != nil
            {
                self.myDepthData = frame.capturedDepthData?.converting(toDepthDataType: kCVPixelFormatType_DepthFloat32)
                self.myDepthDataRaw =  frame.capturedDepthData
                let depthDataMap = self.myDepthData?.depthDataMap
                CVPixelBufferLockBaseAddress(depthDataMap!, CVPixelBufferLockFlags(rawValue: 0))
                let depthPointer = unsafeBitCast(CVPixelBufferGetBaseAddress(depthDataMap!), to: UnsafeMutablePointer<Float>.self)
                self.myModel.image.width = UInt16(CVPixelBufferGetWidth(depthDataMap!))
                self.myModel.image.height = UInt16(CVPixelBufferGetHeight(depthDataMap!))
                self.myModel.camera.width = UInt16(CVPixelBufferGetWidth(depthDataMap!))
                self.myModel.camera.height = UInt16(CVPixelBufferGetHeight(depthDataMap!))
                /*
                 * We have to convert depthDataMap into an UIImage to perform some pre-processing like filtering.
                 */
                //compCIImage(depthDataMap: depthDataMap!)
                //myDepthImage = UIImage(ciImage: myCIImage!)
                //depthView.image = myDepthImage
                //self.myDepthImage.update(_data: depthPointer)
                //self.myDepthImage.update(_data: depthPointer)
                //self.myCamera.update(extrinsics: frame.camera.transform)
                
                //tx.text = "\(self.myDepthData?.cameraCalibrationData?.extrinsicMatrix.columns.3.x)"
                //ty.text = "\(self.myDepthData?.cameraCalibrationData?.extrinsicMatrix.columns.3.x)"
                //tz.text = "\(self.myDepthData?.cameraCalibrationData?.extrinsicMatrix.columns.3.x)"
                
                self.myModel.image.push(map: depthPointer)
                //self.myCamera.update(extrinsics: frame.camera.transform)
                //self.myCamera.intrinsics = frame.camera.intrinsics
                self.numberOfIterations += 1
                /*
                 tx.text = "\(Rt.columns.3.x)"
                 ty.text = "\(Rt.columns.3.y)"
                 tz.text = "\(Rt.columns.3.z)"
                 */
                let last_points = self.myModel.image.data
                //self.myDepthImage.updateDataWithSavedData()
                self.myModel.update(data: depthPointer)
                let current_points = self.myModel.image.data
                var K   = self.myModel.camera.intrinsics
                var Rt  = self.myModel.camera.extrinsics
                bridge_fast_icp(last_points, current_points, &K, &Rt, Int32(self.myModel.camera.width), Int32(self.myModel.camera.height))
                self.myModel.update(extrinsics: Rt, onlyRotation: false)
                
                /* Another Way : Save all depth maps and pose then do it offline */
                //save(model: self.myModel, atTime: k)
                //k += 1
                
            }
            if self.numberOfIterations >= 6
            {
                //let last_points = self.myDepthImage.data
                self.myModel.createMedianDepthMap()
                //let current_points = self.myDepthImage.data
                //var K = self.myCamera.intrinsics
                //var Rt = self.myCamera.extrinsics
                //bridge_fast_icp(last_points, current_points, &K, &Rt, Int32(self.myCamera.width), Int32(self.myCamera.height))
                //self.myCamera.extrinsics = Rt
                //DispatchQueue.global().async {
                self.myModel.integrate()
                //self.myVolume.integrateDepthMap(
                //    image: self.myDepthImage,
                //    camera: self.myCamera,
                //    parameters: [Float(delta), Float(epsilon), Float(lambda)])
                //}
                self.numberOfIterations = 0
            }
        }
    }
            /*
            if self.myDepthImage.savedData.count == 6 {
                DispatchQueue.global().asyncAfter(deadline: .now()+Double(3*self.numberOfIterations)) {
                    if self.numberOfIterations < 10  {
                        self.myDepthImage.updateDataWithSavedData()
                        self.myDepthImage.savedData.removeAll()
                        self.myVolume.integrateDepthMap(
                            image: self.myDepthImage,
                            camera: self.myCamera,
                            parameters: [Float(delta), Float(epsilon), Float(lambda)])
                        self.numberOfIterations += 1
                    }
                    else {
                        self.displayAlertMessage(
                            title: "Fin de l'acquisition",
                            message: "Vous pouvez visualiser avec le bouton \"Show Vol.\"",
                            handler: { _ in
                                self.numberOfIterations = 0
                                self.inRealTime = false
                                self.hasIntegratingStarted = false
                        })
                    }
                }
            }
        }
    }*/
    
    
    @IBAction func startCompute(_ sender: Any) {
        //let group = DispatchGroup()
        integrationProgress.progress = 0.0
        integrationProgress.isHidden = false
        inRealTime = false
        self.myModel.switchTo(realTime: inRealTime)
        self.myModel.reinitExtrinsics()
        
        timer = Double(CFAbsoluteTimeGetCurrent())
        for i in 0..<self.sizeOfDataset {
            //DispatchQueue.main.asyncAfter(deadline: .now() + Double(3*i)) {
                let extrinsics = importCameraPose(from: "frame-\(i).pose", at: self.nameOfDataset)
                var depthmap = importDepthMapFromTXT(from: "frame-\(i).depth", at: self.nameOfDataset)
                bridge_median_filter(&depthmap, 2, Int32(self.myModel.camera.width), Int32(self.myModel.camera.height))
                
                self.myModel.update(extrinsics: extrinsics, onlyRotation: true)
                let last_points = self.myModel.image.data
                self.myModel.update(data: depthmap)
                if i > 0 {
                    /*
                    let current_points = self.myModel.image.data
                    var K = self.myModel.camera.intrinsics
                    var Rt = self.myModel.camera.extrinsics
                    bridge_fast_icp(last_points,
                                    current_points,
                                    &K,
                                    &Rt,
                                    Int32(self.myModel.camera.width),
                                    Int32(self.myModel.camera.height))
                    DispatchQueue.main.async {
                        self.tx.text = "\(Rt.columns.3.x) vs \(extrinsics.columns.3.x)"
                        self.ty.text = "\(Rt.columns.3.y) vs \(extrinsics.columns.3.y)"
                        self.tz.text = "\(Rt.columns.3.z) vs \(extrinsics.columns.3.z)"
                    }
                    /*
                     bridge_drift_correction(current_points,
                     &K,
                     &Rt,
                     myModel.voxels,
                     Int32(self.myModel.numberOfVoxels),
                     myModel.fullResolution(),
                     Int32(self.myModel.camera.width),
                     Int32(self.myModel.camera.height))
                     */
                    //self.myModel.camera.extrinsics.columns.3 = Rt.columns.3
                    //self.myModel.update(extrinsics: Rt, onlyRotation: false)
                    */
                }
                self.myModel.integrate()
                self.integrationProgress.progress = Float(i) / Float(self.sizeOfDataset)
            //}
        }
        let end = Double(CFAbsoluteTimeGetCurrent()) - self.timer
        self.displayAlertMessage(title: "Fin Acquisition", message: "\(end)", handler: {_ in
            self.integrationProgress.isHidden = true
            self.integrationProgress.progress = 0.0
        })
    }
    
    @IBAction func updateEpsilon(_ sender: Any) {
        let quantity = epsilonStepper.value * epsilonTick
        epsilonLabel.text = "Epsilon: \(quantity)"
        myModel.parameters["Epsilon"] = Float(quantity)
    }
    
    @IBAction func updateDelta(_ sender: Any) {
        let quantity = deltaStepper.value * deltaTick
        deltaLabel.text = "Delta: \(quantity)"
        myModel.parameters["Delta"] = Float(quantity)
    }
    
    @IBAction func increaseVolume(_ sender: Any) {
        let quantity = pow(2.0, stepperSize.value)
        volumeSize.text = "Volume Size : \(quantity)"
        myModel.reallocateVoxels(with: Int(quantity))
    }
    
    @IBAction func updateDimension(_ sender: Any) {
        let quantity = dimensionStepper.value * lambdaTick
        dimensionLabel.text = "Vision Max: \(quantity / 2.0)"
        myModel.dimension = Float(quantity)
    }
    
    @IBAction func changeDataset(_ sender: Any) {
        switch datasetChoice.selectedSegmentIndex {
        case 0:
            nameOfDataset = "chair"
        case 1:
            nameOfDataset = "ikea-table"
        default: break
        }
    }
    
    @IBAction func updateDatasetSize(_ sender: Any) {
        let size = Int(datasetSizeField.text!)
        sizeOfDataset = size!
        tx.text = datasetSizeField.text
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
    
    @IBAction func exportVolume(_ sender: Any) {
        self.myModel.switchTo(realTime: true)
        self.displayAlertMessage(
            title: "Acquisition en temps réel",
            message: "Veuillez mettre la caméra en face de l'objet",
            handler: { _ in
                AudioServicesPlaySystemSound (self.systemSoundID)
                self.inRealTime = !self.inRealTime
        })
    }
    
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        if segue.identifier == "ScenePoints" {
            if let destination = segue.destination as? SceneViewController {
                destination.volume = self.myModel
                destination.savedDatasetIndex       = datasetChoice.selectedSegmentIndex
                destination.savedFramesIndex        = datasetSize.selectedSegmentIndex
                destination.savedDeltaIndex         = deltaStepper.value
                destination.savedEpsilonIndex       = epsilonStepper.value
                destination.savedVolumeSizeIndex    = stepperSize.value
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
