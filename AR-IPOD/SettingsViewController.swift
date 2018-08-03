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

class SettingsViewController: UIViewController, UITextFieldDelegate {
    
    @IBOutlet var volumeSize: UITextField!
    @IBOutlet var rangeVision: UITextField!
    @IBOutlet var epsilon: UITextField!
    @IBOutlet var delta: UITextField!
    
    @IBOutlet var icpMaxDistance: UITextField!
    @IBOutlet var icpMaxIterations: UITextField!
    @IBOutlet var icpCorrespondanceDist: UITextField!
    
    @IBOutlet var raytracingSwitch: UISwitch!
    
    var model = Model.sharedInstance
    
    @IBAction func isRaytracingEnable(_ sender: Any) {
        //model.raytracingEnable = raytracingSwitch.isOn
        Model.sharedInstance.raytracingEnable = raytracingSwitch.isOn
    }
    
    @IBAction func setVolumeResolution(_ sender: Any) {
        if let text = volumeSize.text {
            if let value = Int(text) {
                //model.reallocateVoxels(amount: value)
                Model.sharedInstance.reallocateVoxels(amount: value)
            }
        }
    }
    
    @IBAction func setVisionRange(_ sender: Any) {
        if let text = rangeVision.text {
            if let value = Float(text) {
                //model.voxelResolution = value
                Model.sharedInstance.voxelResolution = value
            }
        }
    }
    
    @IBAction func setTruncationDistance(_ sender: Any) {
        if let text = delta.text {
            if let value = Float(text) {
                //model.parameters["Delta"] = value
                Model.sharedInstance.parameters["Delta"] = value
            }
        }
    }
    
    @IBAction func setCarvingDistance(_ sender: Any) {
        if let text = epsilon.text {
            if let value = Float(text) {
                //model.parameters["Epsilon"] = value
                Model.sharedInstance.parameters["Epsilon"] = value
            }
        }
    }
    
    @IBAction func setICPMaximumDistance(_ sender: Any) {
        if let text = icpMaxDistance.text {
            if let value = Float(text) {
                Model.sharedInstance.parameters["icpMaxDist"] = value
            }
        }
    }
    
    @IBAction func setICPMaxIterations(_ sender: Any) {
        if let text = icpMaxIterations.text {
            if let value = Float(text) {
                Model.sharedInstance.parameters["icpMaxIter"] = value
            }
        }
    }
    
    @IBAction func setICPCorrespondanceDist(_ sender: Any) {
        if let text = icpCorrespondanceDist.text {
            if let value = Float(text) {
                Model.sharedInstance.parameters["icpMaxCorr"] = value
            }
        }
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        volumeSize.delegate = self
        rangeVision.delegate = self
        epsilon.delegate = self
        delta.delegate = self
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
                destination.model = Model.sharedInstance
            }
        }
    }
    
    func textFieldShouldReturn(_ textField: UITextField) -> Bool {
        
        textField.resignFirstResponder()
        return true
    }
}
