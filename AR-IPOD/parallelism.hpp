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

void bridge_initializeCentroids(void* centroids,
                                int size,
                                float resolution);

unsigned long bridge_extractMesh(void* triangles,
                            const float* voxels,
                            const void* centroids,
                            int edgeTable[256],
                            int triTable[4096],
                            int n,
                            float isolevel);

int bridge_integrateDepthMap(const float* depthmap,
                              const void* centroids,
                              const void* camera_pose,
                              const void* intrisics,
                              void* voxels,
                              const int width,
                              const int height);

#endif /* volumeInit_hpp */
