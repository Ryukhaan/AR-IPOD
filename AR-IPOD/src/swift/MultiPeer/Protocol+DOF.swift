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
    func transformChanged(manager : DOFServiceManager, transform: matrix_float4x4)
    func integrationFinished(manager: DOFServiceManager)
    func startIntegrating(manager: DOFServiceManager)
    func resetModel(manager: DOFServiceManager)
}
