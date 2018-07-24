//
//  Extension+DOF.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 11/07/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import MultipeerConnectivity
import ARKit

extension DOFServiceManager : MCNearbyServiceAdvertiserDelegate {
    
    func advertiser(_ advertiser: MCNearbyServiceAdvertiser, didNotStartAdvertisingPeer error: Error) {
        NSLog("%@", "didNotStartAdvertisingPeer: \(error)")
    }
    
    func advertiser(_ advertiser: MCNearbyServiceAdvertiser, didReceiveInvitationFromPeer peerID: MCPeerID, withContext context: Data?, invitationHandler: @escaping (Bool, MCSession?) -> Void) {
        NSLog("%@", "didReceiveInvitationFromPeer \(peerID)")
        invitationHandler(true, self.mySession)
    }
}

extension DOFServiceManager : MCNearbyServiceBrowserDelegate {
    
    func browser(_ browser: MCNearbyServiceBrowser, didNotStartBrowsingForPeers error: Error) {
        NSLog("%@", "didNotStartBrowsingForPeers: \(error)")
    }
    
    func browser(_ browser: MCNearbyServiceBrowser, foundPeer peerID: MCPeerID, withDiscoveryInfo info: [String : String]?) {
        NSLog("%@", "foundPeer: \(peerID)")
        NSLog("%@", "invitePeer: \(peerID)")
        browser.invitePeer(peerID, to: self.mySession, withContext: nil, timeout: 10)
    }
    
    func browser(_ browser: MCNearbyServiceBrowser, lostPeer peerID: MCPeerID) {
        NSLog("%@", "lostPeer: \(peerID)")
    }
}

extension DOFServiceManager : MCSessionDelegate {
    
    // Display connected devices (ViewController)
    func session(_ session: MCSession, peer peerID: MCPeerID, didChange state: MCSessionState) {
        NSLog("%@", "peer \(peerID) didChangeState: \(state)")
        self.delegate?.connectedDevicesChanged(manager: self, connectedDevices:
            session.connectedPeers.map{$0.displayName})
    }
    
    // Change color background (ViewController)
    func session(_ session: MCSession, didReceive data: Data, fromPeer peerID: MCPeerID) {
        //NSLog("%@", "didReceiveData: \(data)")
        let str = String(data: data, encoding: .utf8)!
        let rows = str.components(separatedBy: .whitespacesAndNewlines)
        var buffer = [Float](repeating: 0, count: 16)
        for (i, row) in rows.enumerated() {
            buffer[i] = Float(row)!
        }
        let M = matrix_float4x4(rows: [float4(buffer[0], buffer[1], buffer[2], buffer[3]),
                                       float4(buffer[4], buffer[5], buffer[6], buffer[7]),
                                       float4(buffer[8], buffer[9], buffer[10], buffer[11]),
                                       float4(buffer[12], buffer[13], buffer[14], buffer[15])
            ])
        //let coxy = CGSize(width: CGFloat(buffer[1]), height: CGFloat(buffer[2]))
        switch rows[0] {
        case Constant.Code.Integration.hasFinished:
            self.delegate?.integrationFinished(manager: self)
        case Constant.Code.Integration.isStarting:
            self.delegate?.startIntegrating(manager: self)
        case Constant.Code.Integration.reset:
            self.delegate?.resetModel(manager: self)
        //case Constant.Code.Photo.sendingPhoto:
            //self.delegate?.photoChaned(manager: self)
        default:
            self.delegate?.transformChanged(manager: self, transform: M)
        }
    }
    
    func session(_ session: MCSession, didReceive stream: InputStream, withName streamName: String, fromPeer peerID: MCPeerID) {
        NSLog("%@", "didReceiveStream")
    }
    
    func session(_ session: MCSession, didStartReceivingResourceWithName resourceName: String, fromPeer peerID: MCPeerID, with progress: Progress) {
        NSLog("%@", "didStartReceivingResourceWithName")
    }
    
    func session(_ session: MCSession, didFinishReceivingResourceWithName resourceName: String, fromPeer peerID: MCPeerID, at localURL: URL?, withError error: Error?) {
        NSLog("%@", "didFinishReceivingResourceWithName")
    }
}
