//
//  Tables.swift
//  AR-IPOD
//
//  Created by Remi Decelle on 26/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

import Foundation
import ARKit

struct Tables {
    static let edgeTable: [Int]  = {
        var text: String = ""
        if let path = Bundle.main.path(forAuxiliaryExecutable: "EdgeTableContents") {
            do {
                try text = String(contentsOfFile: path, encoding: .utf8)
            }
            catch {
                print("Sorry could not load file EdgeTableContents.rtf !")
            }
        }
        let preprocess = text.components(separatedBy: .whitespacesAndNewlines).joined()
            //.components(separatedBy: " ").joined() // Remove spaces
            .components(separatedBy: "0x").joined() // Remove 0x
        return preprocess.components(separatedBy: ",").map { Int($0, radix: 16)! }
    }()
    
    static let triTable: [[Int]] = {
        var text: String = ""
        if let path = Bundle.main.path(forAuxiliaryExecutable: "TriTableContents") {
            do {
                try text = String(contentsOfFile: path, encoding: .utf8)
            }
            catch {
                print("Sorry could not load file TriTableContents.rtf !")
            }
        }
        return text.components(separatedBy: .newlines).map {
            $0.components(separatedBy: " ").joined().components(separatedBy: ",").map { Int($0)! }
        }
    }()
}
