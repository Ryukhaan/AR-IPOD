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

typedef struct m_float3{
    float x, y, z;
} m_float3;

#ifdef __cplusplus
extern "C" {
#endif
    
    void* bridge_extractMesh(
                            int* n_triangles,
                            const void* voxels,
                            const int edgeTable[256],
                            const int triTable[4096],
                            const int dimension,
                            const float isolevel,
                            const float resolution);
    
#ifdef __cplusplus
}
#endif

void bridge_integrateDepthMap(const float* depthmap,
                              //const void* centroids,
                              const void* rotation,
                              const void* translation,
                              const void* intrinsics,
                              void* voxels,
                              const int width,
                              const int height,
                              const int dimension,
                              const float resolution,
                              const float delta,
                              const float epsilon,
                              const float lambda,
                              const float cx,
                              const float cy);

void bridge_exportMeshToPLY(const void* vectors,
                            const char* file_name,
                            int n);

void bridge_exportVolumeToPLY(const void* centroids,
                              const float* sdfs,
                              const char* file_name,
                              int size);


void bridge_median_filter(float* depthmap,
                          const int window_size,
                          const int width,
                          const int height);

void bridge_drift_correction(const float* current_points,
                             const void* intrinsics,
                             void* extrinsics,
                             const void* voxels,
                             const int dimension,
                             const float resolution,
                             const int width,
                             const int height);

#endif /* volumeInit_hpp */
