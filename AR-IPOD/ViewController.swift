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

    var sizeOfDataset: Int = 1
    var dataset: String = "chair"
    var myVolume: Volume = Volume.sharedInstance
    var depthImage: DepthImage = DepthImage()
    var myCamera: Camera = Camera()
    var increments: Int = 1
    var timer = Double(CFAbsoluteTimeGetCurrent())
    var inRealTime : Bool = false
    var startIntegrator: Bool = false
    
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
        let configuration = ARWorldTrackingConfiguration()
        //let configuration = ARFaceTrackingConfiguration()

        // Run the view's session
        sceneView.session.run(configuration)
        
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
        guard let currentFrame = sceneView.session.currentFrame
            else { return }
        if startIntegrator {
            if currentFrame.capturedDepthData != nil {
                myDepthData = currentFrame.capturedDepthData?.converting(toDepthDataType: kCVPixelFormatType_DepthFloat32)
                myDepthDataRaw =  currentFrame.capturedDepthData
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
                self.depthImage.update(_data: depthPointer)
            }
            let epsilon = epsilonStepper.value * epsilonTick
            let delta = deltaStepper.value * deltaTick
            myCamera.update(extrinsics: currentFrame.camera.transform)
            myVolume.integrateDepthMap(image: self.depthImage, camera: &self.myCamera, parameters: [Float(delta), Float(epsilon)])
        }
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
        if startIntegrator {
            if frame.capturedDepthData != nil {
                myDepthData = frame.capturedDepthData?.converting(toDepthDataType: kCVPixelFormatType_DepthFloat32)
                myDepthDataRaw =  frame.capturedDepthData
                let depthDataMap = myDepthData?.depthDataMap
                CVPixelBufferLockBaseAddress(depthDataMap!, CVPixelBufferLockFlags(rawValue: 0))
                let depthPointer = unsafeBitCast(CVPixelBufferGetBaseAddress(depthDataMap!), to: UnsafeMutablePointer<Float>.self)
                /*
                 * Potential Pre-processing : Median Filter.
                 * We have to convert depthDataMap into an UIImage to do this. */
                //compCIImage(depthDataMap: depthDataMap!)
                //myDepthImage = UIImage(ciImage: myCIImage!)
                //depthView.image = myDepthImage
 
                self.depthImage.update(_data: depthPointer)
            }
            let epsilon = epsilonStepper.value * epsilonTick
            let delta = deltaStepper.value * deltaTick
            myCamera.update(extrinsics: frame.camera.transform)
            myVolume.integrateDepthMap(image: self.depthImage, camera: &self.myCamera, parameters: [Float(delta), Float(epsilon)])
        }
         */
    }
    
    
    @IBAction func startCompute(_ sender: Any) {
        if !inRealTime {
            let epsilon = epsilonStepper.value * epsilonTick
            let delta = deltaStepper.value * deltaTick
            for i in 0..<self.sizeOfDataset {
                let extrinsics = importCameraPose(from: "frame-\(i).pose", at: dataset)
                let depthmap = importDepthMapFromTXT(from: "frame-\(i).depth", at: dataset)
                self.myCamera.update(extrinsics: extrinsics)
                self.depthImage.update(_data: depthmap)
                self.myVolume.integrateDepthMap(image: self.depthImage, camera: &self.myCamera, parameters: [Float(delta), Float(epsilon)])
            }
        }
        else {
            let alert = UIAlertController(title: "Acquisition en temps réel", message: "Veuillez mettre la front-camera en face de l'objet à aquérir", preferredStyle: .alert)
             alert.addAction(UIAlertAction(title: "OK", style: .default, handler: { [weak alert] (_) in
                print("Début de l'acquisition")}
            ))
            self.present(alert, animated: true, completion: nil)
            startIntegrator = true
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
        case 2:
            inRealTime = true
        default: break
        }
    }
    @IBAction func changeDatasetSize(_ sender: Any) {
        switch datasetSize.selectedSegmentIndex {
        case 0:
            sizeOfDataset = 1
        case 1:
            sizeOfDataset = 10
        case 2:
            sizeOfDataset = 100
        default: break
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
        let delta = deltaStepper.value * deltaTick
        let points = extractMesh(volume: myVolume, isolevel: Float(isolevel))
        //let points = extractMesh(volume: myVolume, isolevel: Float(delta))
        //sceneView.scene.rootNode.addChildNode(pointNode)
        exportToPLY(mesh: points, at: "mesh_\(dataset)_\(self.myVolume.numberOfVoxels).ply")
        //exportToPLY(volume: self.myVolume, at: "volume_\(dataset)_\(self.myVolume.numberOfVoxels).ply")
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
}
