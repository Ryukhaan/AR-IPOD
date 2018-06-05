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
#include <sys/stat.h>
#include <cstring>
#include <cerrno>

using namespace std;

inline float integer_to_global(float point, int dim, float resolution) {
    return (resolution / dim) * (point + 0.5);
}

inline simd_float3 centroid_at(const int i,
                               const int dimension,
                               const float resolution,
                               const int square,
                               const float offset) {
    simd::float3 centroid;
    int x = i / square;
    int remainder = i % square;
    int y = remainder / dimension;
    int z = remainder % dimension;
    centroid.x = integer_to_global(x, dimension, resolution) - offset;
    centroid.y = integer_to_global(y, dimension, resolution) - offset;
    centroid.z = integer_to_global(z, dimension, resolution);// - offset;
    return centroid;
}

void save_meshing_ply_format(const simd::float3* points, const char* file_name, int number_of_triangles) {
    std::ofstream file;
    /* file_name + 7 because file_name is like file:///var/... ] */
    file.open(file_name + 7, std::ofstream::trunc);
    if ( ! file.is_open() ) return;
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
        file << t1.x << " " << t1.y << " " << t1.z << endl;
        file << t2.x << " " << t2.y << " " << t2.z << endl;
        file << t3.x << " " << t3.y << " " << t3.z << endl;
    }
    
    // Write triangles
    for(int i = 0; i<number_of_triangles; i++) {
        file << "3 " << (3*i) << " " << (3*i+1) << " " << (3*i+2) << endl;
    }
    file.flush();
    file.close();
}

void save_volume_ply_format(const simd::float3* centroids, const float* sdfs, const char* file_name, int size) {
    std::ofstream file;
    file.open(file_name + 7, std::ofstream::trunc);
    if ( ! file.is_open() ) return;
    // Write PLY Header
    file << "ply" << endl;
    file << "format ascii 1.0" << endl;
    file << "element vertex " << size << endl;
    file << "property float x" << endl;
    file << "property float y" << endl;
    file << "property float z" << endl;
    file << "property float red" << endl;
    file << "property float green" << endl;
    file << "property float blue" << endl;
    file << "end_header" << endl;
    
    // Write Vertex
    for(int i = 0; i<size; i++) {
        //simd::float3 centroid = centroids[i];
        simd::float3 centroid = centroid_at(i, 256, 3.0, 65536, 1.5);
        float sdf = sdfs[i];
        file << centroid.x << " " << centroid.y << " " << centroid.z << " ";
        file << sdf << " 54 14" << endl;
    }
    file.flush();
    file.close();
}
