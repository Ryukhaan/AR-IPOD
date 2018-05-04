//
//  io.cpp
//  AR-IPOD
//
//  Created by Remi Decelle on 04/05/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#include "io.hpp"
#include <iostream>
#include <limits>
#include <fstream>

using namespace std;

void save_meshing_ply_format(const simd::float3* points, const char* file_name, int number_of_triangles) {
    std::ofstream file;
    file.open(file_name);
    // Write PLY Header
    file << "ply" << endl;
    file << "format ascii 1.0" << endl;
    file << "element vertex " << (3*number_of_triangles) << endl;
    file << "property float x" << endl;
    file << "property float y" << endl;
    file << "property float z" << endl;
    file << "element face " << number_of_triangles << endl;
    file << "property list uchar int vertex_index" << endl;
    file << "end_header" << endl;
    
    // Write Vertex
    for(int i = 0; i<number_of_triangles; i++) {
        simd::float3 t1 = points[3*i];
        simd::float3 t2 = points[3*i+1];
        simd::float3 t3 = points[3*i+2];
        file << t1.x << " " << t1.y << " " << t1.z << "\n";
        file << t2.x << " " << t2.y << " " << t2.z << "\n";
        file << t3.x << " " << t3.y << " " << t3.z << "\n";
    }
    
    // Write triangles
    for(int i = 0; i<number_of_triangles; i++) {
        file << "3 " << (3*i) << " " << (3*i+1) << " " << (3*i+2) << endl;
    }
    file.flush();
    file.close();
}
