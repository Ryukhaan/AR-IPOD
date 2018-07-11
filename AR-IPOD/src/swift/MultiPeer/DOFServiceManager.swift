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
    
    func send(colorName : String) {
        NSLog("%@", "sendColor: \(colorName) to \(mySession.connectedPeers.count) peers")
        if mySession.connectedPeers.count > 0 {
            do {
                try self.mySession.send(colorName.data(using: .utf8)!, toPeers: mySession.connectedPeers, with: .reliable)
            }
            catch let error {
                NSLog("%@", "Error for sending: \(error)")
            }
        }
    }
    
    func send(imageURL: URL) {
        NSLog("%@", "send new depthmap to \(mySession.connectedPeers.count) peers")
        if mySession.connectedPeers.count > 0 {
            // Need to iterate over connected peers
            for id in mySession.connectedPeers {
                //try self.mySession.send(image, toPeers: mySession.connectedPeers, with: .reliable)
                self.mySession.sendResource(at: imageURL,
                                            withName: "depth.jpg",
                                            toPeer: id,
                                            withCompletionHandler: nil)
            }
        }
    }
    
    func send(transform M: matrix_float4x4) {
        let datas = """
        \(M.columns.0.x) \(M.columns.1.x) \(M.columns.2.x) \(M.columns.3.x)
        \(M.columns.0.y) \(M.columns.1.y) \(M.columns.2.y) \(M.columns.3.y)
        \(M.columns.0.z) \(M.columns.1.z) \(M.columns.2.z) \(M.columns.3.z)
        \(M.columns.0.w) \(M.columns.1.w) \(M.columns.2.w) \(M.columns.3.w)
        """
        NSLog("%@", "send camera position : \n \(datas) \n to \(mySession.connectedPeers.count) peers")
        if mySession.connectedPeers.count > 0 {
            do {
                try self.mySession.send(datas.data(using: .utf8)!, toPeers: mySession.connectedPeers, with: .reliable)
            }
            catch let error {
                NSLog("%@", "Error for sending: \(error)")
            }
        }
    }
}

