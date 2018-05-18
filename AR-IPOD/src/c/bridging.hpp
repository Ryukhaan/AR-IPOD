//
//  volumeInit.hpp
//  AR-IPOD
//
//  Created by Remi Decelle on 27/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#ifndef volumeInit_hpp
#define volumeInit_hpp
#include <stdio.h>
#include <string.h>

void bridge_initializeCentroids(void* centroids,
                                int size,
                                float resolution);

unsigned long bridge_extractMesh(void* triangles,
                                 void* voxels,
                                 //const float* voxels,
                                 //const void* centroids,
                                 int edgeTable[256],
                                 int triTable[4096],
                                 int n,
                                 float isolevel);

int bridge_integrateDepthMap(float* depthmap,
                             //const void* centroids,
                             const void* extrinsics,
                             const void* intrinsics,
                             void* voxels,
                             const int width,
                             const int height,
                             const int dimension,
                             const float resolution[3],
                             const float delta,
                             const float epsilon,
                             const float lambda);

void bridge_exportMeshToPLY(const void* vectors,
                            const char* file_name,
                            int n);

void bridge_exportVolumeToPLY(const void* centroids,
                              const float* sdfs,
                              const char* file_name,
                              int size);

void bridge_fast_icp(const float* last_points,
                     const float* current_points,
                     const void* intrinsics,
                     void* extrinsics,
                     const int width,
                     const int height);

#endif /* volumeInit_hpp */
