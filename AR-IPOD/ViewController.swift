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

class ViewController: UIViewController, ARSCNViewDelegate, ARSessionDelegate {
    
    @IBOutlet var tx: UILabel!
    @IBOutlet var ty: UILabel!
    @IBOutlet var tz: UILabel!
    
    @IBOutlet var sceneView: ARSCNView!
    
    let myARSession: ARSession = ARSession()
    let systemSoundID: SystemSoundID = 1016
    var myVolume: Volume            = Volume.sharedInstance
    var myDepthImage: DepthImage    = DepthImage(onRealTime: false)
    var myCamera: Camera            = Camera(onRealTime: false)
    
    var sizeOfDataset: Int          = 1
    var nameOfDataset: String       = "chair"
    var numberOfIterations: Int     = 0
    var timer                       = Double(CFAbsoluteTimeGetCurrent())
    var inRealTime: Bool            = false
    var hasIntegratingStarted: Bool = false
    
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
    @IBOutlet var lambdaLabel: UILabel!
    @IBOutlet var lambdaStepper: UIStepper!
    let lambdaTick = 0.02
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
        let tracking = AROrientationTrackingConfiguration()
        let configuration = ARFaceTrackingConfiguration()
        
        // Run the view's session
        sceneView.session.run(configuration)
        sceneView.session.delegate = self
        
        myARSession.run(tracking)
        myARSession.delegate = self
        
        // Version with dataset
        let starter = Double(CFAbsoluteTimeGetCurrent())
        let threads = [DispatchQueue(label: "thread1", qos: .userInteractive, attributes: .concurrent),
                       DispatchQueue(label: "thread2", qos: .userInteractive, attributes: .concurrent)
        ]
        let group = DispatchGroup()
        group.enter()
        threads[0].async {
            let intrinsics = importCameraIntrinsics(from: "depthIntrinsics", at: self.nameOfDataset)
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
        let epsilon = epsilonStepper.value * epsilonTick
        let delta = deltaStepper.value * deltaTick
        let lambda = lambdaStepper.value * lambdaTick
        integrationProgress.progress = 0.0
        integrationProgress.isHidden = false
        if hasIntegratingStarted
        {
            tx.text = "\(myARSession.currentFrame?.camera.transform.columns.3.x)"//"\(self.myCamera.extrinsics.columns.3.x)"
            ty.text = "\(myARSession.currentFrame?.camera.transform.columns.3.y)"//"\(self.myCamera.extrinsics.columns.3.y)"
            tz.text = "\(myARSession.currentFrame?.camera.transform.columns.3.z)"//"\(self.myCamera.extrinsics.columns.3.z)"
            if frame.capturedDepthData != nil
            {
                self.myDepthData = frame.capturedDepthData?.converting(toDepthDataType: kCVPixelFormatType_DepthFloat32)
                self.myDepthDataRaw =  frame.capturedDepthData
                let depthDataMap = self.myDepthData?.depthDataMap
                CVPixelBufferLockBaseAddress(depthDataMap!, CVPixelBufferLockFlags(rawValue: 0))
                let depthPointer = unsafeBitCast(CVPixelBufferGetBaseAddress(depthDataMap!), to: UnsafeMutablePointer<Float>.self)
                self.myDepthImage.width = UInt16(CVPixelBufferGetWidth(depthDataMap!))
                self.myDepthImage.height = UInt16(CVPixelBufferGetHeight(depthDataMap!))
                self.myCamera.width = UInt16(CVPixelBufferGetWidth(depthDataMap!))
                self.myCamera.height = UInt16(CVPixelBufferGetHeight(depthDataMap!))
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
                
                //self.myDepthImage.push(map: depthPointer)
                self.myCamera.update(extrinsics: (myDepthData?.cameraCalibrationData?.extrinsicMatrix)!)
                self.myCamera.intrinsics = (myDepthData?.cameraCalibrationData?.intrinsicMatrix)!
                self.numberOfIterations += 1
            }
            if self.numberOfIterations >= 6
            {
                //self.myDepthImage.updateDataWithSavedData()
                //DispatchQueue.global().async {
                /*
                self.myVolume.integrateDepthMap(
                    image: self.myDepthImage,
                    camera: self.myCamera,
                    parameters: [Float(delta), Float(epsilon), Float(lambda)])
                 */
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
        let group = DispatchGroup()
        integrationProgress.progress = 0.0
        integrationProgress.isHidden = false
        let epsilon = epsilonStepper.value * epsilonTick
        let delta = deltaStepper.value * deltaTick
        let lambda = lambdaStepper.value * lambdaTick
        var itDone = 0
        var itRead = 0
        let integrateItem = DispatchWorkItem {
            // Compute TSDF
            //let extrinsics = importCameraPose(from: "frame-\(itRead).pose", at: self.nameOfDataset)
            let depthmap = importDepthMapFromTXT(from: "frame-\(itRead).depth", at: self.nameOfDataset)
            //self.myCamera.update(extrinsics: extrinsics)
            self.myDepthImage.update(_data: depthmap)
            self.myVolume.integrateDepthMap(image: self.myDepthImage,
                                            camera: self.myCamera,
                                            parameters: [Float(delta), Float(epsilon), Float(lambda)])
            itDone += 1
            // Update UI
            DispatchQueue.main.async {
                self.integrationProgress.progress = Float(itDone) / Float(self.sizeOfDataset)
                let end = Double(CFAbsoluteTimeGetCurrent()) - self.timer
                if itDone == self.sizeOfDataset {
                    self.displayAlertMessage(title: "Fin Acquisition", message: "\(end)", handler: {_ in
                        self.integrationProgress.isHidden = true
                        self.integrationProgress.progress = 0.0
                    })
                }
            }
        }
        
        if datasetChoice.selectedSegmentIndex != DataAcquisition.InRealTime {
            timer = Double(CFAbsoluteTimeGetCurrent())
            for i in 0..<self.sizeOfDataset {
                group.enter()
                DispatchQueue.global().asyncAfter(deadline: .now()+0.1+Double(3*i)) {
                    itRead = i
                    integrateItem.perform()
                    group.leave()
                }
            }
            group.wait()
        }
        else {
            self.numberOfIterations = 0
            self.displayAlertMessage(
                title: "Acquisition en temps réel",
                message: "Veuillez mettre la front-caméra face de l'objet",
                handler: { _ in
                    self.hasIntegratingStarted = true
                    AudioServicesPlaySystemSound (self.systemSoundID)
            })
        }
    }
    
    @IBAction func updateEpsilon(_ sender: Any) {
        let quantity = epsilonStepper.value * epsilonTick
        epsilonLabel.text = "Epsilon: \(quantity)"
    }
    
    @IBAction func updateDelta(_ sender: Any) {
        let quantity = deltaStepper.value * deltaTick
        deltaLabel.text = "Delta: \(quantity)"
    }
    
    @IBAction func increaseVolume(_ sender: Any) {
        let quantity = pow(2.0, stepperSize.value)
        self.myVolume.initialize(with: Int(quantity))
        volumeSize.text = "Volume Size : \(quantity)"
    }
    
    @IBAction func updateLambda(_ sender: Any) {
        let quantity = lambdaStepper.value * lambdaTick
        lambdaLabel.text = "Lambda: \(quantity)"
    }
    
    @IBAction func changeDataset(_ sender: Any) {
        hasIntegratingStarted = false
        inRealTime = false
        switch datasetChoice.selectedSegmentIndex {
        case 0:
            nameOfDataset = "chair"
        case 1:
            nameOfDataset = "ikea-table"
        default: break
        }
        myDepthImage.changeTo(realTime: inRealTime)
        myCamera.changeTo(realTime: inRealTime)
    }
    
    @IBAction func changeDatasetSize(_ sender: Any) {
        sizeOfDataset = Int(pow(10.0, Float(datasetSize.selectedSegmentIndex)))
    }
    
    @IBAction func exportVolume(_ sender: Any) {
        /*
         guard let currentFrame = sceneView.session.currentFrame
         else { return }
         */
        /*
         var isolevel: Float
         if isolevelStepper.value == 0 {
         isolevel = 0
         }
         else {
         isolevel = Float(pow(10.0, isolevelStepper.value - 6))
         }
         //let delta = deltaStepper.value * deltaTick
         let points = extractMesh(volume: myVolume, isolevel: Float(isolevel))
         //let points = extractMesh(volume: myVolume, isolevel: Float(delta))
         //sceneView.scene.rootNode.addChildNode(pointNode)
         exportToPLY(mesh: points, at: "mesh_\(dataset)_\(self.myVolume.numberOfVoxels).ply")
         */
         exportToPLY(volume: self.myVolume, at: "volume_\(self.sizeOfDataset)_\(self.myVolume.numberOfVoxels).ply")
 
    }
    
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        if segue.identifier == "ScenePoints" {
            if let destination = segue.destination as? SceneViewController {
                //destination.volume = self.myVolume
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
