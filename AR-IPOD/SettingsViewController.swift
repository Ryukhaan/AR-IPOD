//
//  SettingsViewController.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 08/06/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import UIKit
import SceneKit

class SettingsViewController: UIViewController {
    
    @IBOutlet var volumeSize: UITextField!
    @IBOutlet var rangeVision: UITextField!
    @IBOutlet var epsilon: UITextField!
    @IBOutlet var delta: UITextField!
    
    @IBOutlet var cameraPoseSwitch: UISwitch!
    @IBOutlet var raytracingSwitch: UISwitch!
    
    var model = Model.sharedInstance
    
    @IBAction func isPoseEstimated(_ sender: Any) {
        model.cameraPoseEstimationEnable = cameraPoseSwitch.isOn
    }
    
    @IBAction func isRaytracingEnable(_ sender: Any) {
        model.raytracingEnable = raytracingSwitch.isOn
    }
    
    @IBAction func setVolumeResolution(_ sender: Any) {
        if let text = volumeSize.text {
            if let value = Int(text) {
                model.reallocateVoxels(with: value)
            }
        }
    }
    
    @IBAction func setVisionRange(_ sender: Any) {
        if let text = rangeVision.text {
            if let value = Float(text) {
                model.voxelResolution = value
            }
        }
    }
    
    @IBAction func setTruncationDistance(_ sender: Any) {
        if let text = delta.text {
            if let value = Float(text) {
                model.parameters["Delta"] = value
            }
        }
    }
    
    @IBAction func setCarvingDistance(_ sender: Any) {
        if let text = epsilon.text {
            if let value = Float(text) {
                model.parameters["Epsilon"] = value
            }
        }
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
    }
    
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        if segue.identifier == "Main" {
            if let destination = segue.destination as? ViewController {
                destination.myModel = self.model
            }
        }
    }
}
