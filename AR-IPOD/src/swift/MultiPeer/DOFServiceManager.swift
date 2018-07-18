//
//  DOFServiceManager.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 11/07/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import MultipeerConnectivity
import ARKit

class DOFServiceManager : NSObject {
    
    // Service type must be a unique string, at most 15 characters long
    // and can contain only ASCII lowercase letters, numbers and hyphens.
    var correctionM = matrix_float4x4(diagonal: float4(1,1,1,1))
    private let DofServiceType = "dof-service"
    
    private let myPeerId = MCPeerID(displayName: UIDevice.current.name)
    private let serviceAdvertiser : MCNearbyServiceAdvertiser
    private let serviceBrowser : MCNearbyServiceBrowser
    
    lazy var mySession : MCSession = {
        let session = MCSession(peer: self.myPeerId, securityIdentity: nil, encryptionPreference: .required)
        session.delegate = self
        return session
    }()
    
    var delegate : DOFServiceManagerDelegate?
    
    override init() {
        self.serviceAdvertiser = MCNearbyServiceAdvertiser(peer: myPeerId, discoveryInfo: nil, serviceType: DofServiceType)
        self.serviceBrowser = MCNearbyServiceBrowser(peer: myPeerId, serviceType: DofServiceType)
        super.init()
        
        self.serviceAdvertiser.delegate = self
        self.serviceAdvertiser.startAdvertisingPeer()
        
        self.serviceBrowser.delegate = self
        self.serviceBrowser.startBrowsingForPeers()
    }
    
    deinit {
        self.serviceAdvertiser.stopAdvertisingPeer()
        self.serviceBrowser.stopBrowsingForPeers()
    }
    
    
    func send(alert A: String) {
        let text = """
        \(A) 0 0 0
        0 0 0 0
        0 0 0 0
        0 0 0 0
        """
        //NSLog("%@", "Integrate depth map done to \(mySession.connectedPeers.count) peers")
        self.send(text: text)
    }
    
    func send(transform A: matrix_float4x4) {

        let reorientIpad = matrix_float4x4([
            float4(0,   -1,    0,  0),
            float4(-1,  0,    0,  0),
            float4(0,   0,    -1,  0),
            float4(0,   0,    0,  1),
            ])
 
        let R = simd_mul(A, simd_transpose(reorientIpad))
        //let R = simd_mul(A, simd_transpose(correctionM))
        let text = """
        \(R.columns.0.x) \(R.columns.1.x) \(R.columns.2.x) \(-A.columns.3.x)
        \(R.columns.0.y) \(R.columns.1.y) \(R.columns.2.y) \(A.columns.3.y)
        \(R.columns.0.z) \(R.columns.1.z) \(R.columns.2.z) \(-A.columns.3.z)
        0 0 0 1
        """
 
        //NSLog("%@", "Send camera position : \n \(text) \n to \(mySession.connectedPeers.count) peers")
        self.send(text: text)
    }
    
    private func send(text: String) {
        if mySession.connectedPeers.count > 0 {
            do {
                try self.mySession.send(text.data(using: .utf8)!, toPeers: mySession.connectedPeers, with: .reliable)
            }
            catch let error {
                NSLog("%@", "Error for sending: \(error)")
            }
        }
    }
    
    private func transferFile(file: URL) {
        if mySession.connectedPeers.count > 0 {
            for id in mySession.connectedPeers {
                self.mySession.sendResource(at: file, withName: "photo.jpg", toPeer: id, withCompletionHandler: nil)
            }
        }
    }
}

