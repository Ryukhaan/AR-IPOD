//
//  Protocol+DOF.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 11/07/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import MultipeerConnectivity
import ARKit

protocol DOFServiceManagerDelegate {
    
    func connectedDevicesChanged(manager : DOFServiceManager, connectedDevices: [String])
    func colorChanged(manager : DOFServiceManager, colorString: String)
    
    func imageChanged(manager : DOFServiceManager, image: UIImage)
    func transformChanged(manager : DOFServiceManager, transform: matrix_float4x4)
}
