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

unsigned long bridge_extractMesh(void* triangles,
                                 void* voxels,
                                 const int edgeTable[256],
                                 const int triTable[4096],
                                 const int dimension,
                                 const float isolevel,
                                 const float resolution);

void bridge_integrateDepthMap(float* depthmap,
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
                             const float lambda);

void bridge_exportMeshToPLY(const void* vectors,
                            const char* file_name,
                            int n);

void bridge_exportVolumeToPLY(const void* centroids,
                              const float* sdfs,
                              const char* file_name,
                              int size);


void bridge_global_registration(const float* previous,
                                const float* current,
                                //const void* voxels,
                                const int width,
                                const int height,
                                void* rotation,
                                void* translation,
                                const void* intrinsics,
                                const float resolution,
                                const int dimension,
                                const float thresh_depth,
                                const float corresp_dist,
                                const int max_num_iter);

/*
void bridge_global_registration(const float* previous,
                                const float* current,
                                const void* voxels,
                                const int width,
                                const int height,
                                void* rotation,
                                void* translation,
                                const void* intrinsics,
                                const float resolution,
                                const int dimension);
*/

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

void bridge_raycastDepthMap(float* depthmap,
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
                            const float lambda);
#endif /* volumeInit_hpp */
