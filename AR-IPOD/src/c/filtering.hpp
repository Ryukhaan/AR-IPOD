//
//  filtering.hpp
//  AR-IPOD
//
//  Created by Remi Decelle on 15/05/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#ifndef filtering_hpp
#define filtering_hpp

#include <stdio.h>
#include <simd/common.h>
#include <simd/vector_types.h>
#include <simd/vector_make.h>
#include <simd/vector.h>
#include <simd/conversion.h>
#include <simd/matrix.h>
#include <vector>
#include <algorithm>

#define KINECT_WIDTH 640
#define KINECT_HEIGHT 480

void median_filter(float* depthmap,
                   const int window_size,
                   const int width,
                   const int height) {
    float* tmp_depth = (float*) malloc(sizeof(float) * width * height);
    //float tmp_depth[ KINECT_WIDTH * KINECT_HEIGHT ];
    for (int i = 0; i<height; i++) {
        int iMin = fmax(i-window_size, 0);
        int iMax = fmin(i+window_size, height-1);
        for (int j = 0; j<width; j++) {
            int jMin = fmax(j-window_size, 0);
            int jMax = fmin(j+window_size, width-1);
            std::vector<float> tmp_array;
            for (int k = iMin; k<=iMax; k++) {
                for (int l = jMin; l<=jMax; l++) {
                    tmp_array.push_back(depthmap[k*width + l]);
                }
            }
            std::sort(tmp_array.begin(), tmp_array.end());
            int size = static_cast<int>( tmp_array.size() );
            if ( size % 2 == 0 )
                tmp_depth[i*width + j] = tmp_array[size/2];
            else
                tmp_depth[i*width + j] = (tmp_array[(size+1)/2.0] + tmp_array[(size-1)/2.0])/2.0;
        }
    }
    for (int i = 0; i< height * width ; i++)
        depthmap[i] = tmp_depth[i];
    free(tmp_depth);
    
}

/*
void local_median(float* depthmap,
                  const int window_size,
                  const int width,
                  const int height,
                  const int i,
                  const int j) {
    int iMin = fmax(i-window_size, 0);
    int iMax = fmin(i+window_size, height-1);
    int jMin = fmax(j-window_size, 0);
    int jMax = fmin(j+window_size, width-1);
    std::vector<float> tmp_array;
    for (int k = iMin; k<=iMax; k++)
        for (int l = jMin; l<=jMax; l++)
            tmp_array.push_back(depthmap[k*width + l]);
    std::sort(tmp_array.begin(), tmp_array.end());
    int size = static_cast<int>( tmp_array.size() );
    if ( size % 2 != 0 )
        depthmap[i*width + j] = tmp_array[size/2];
    else
        depthmap[i*width + j] = (tmp_array[(size+1)/2.0] + tmp_array[(size-1)/2.0])/2.0;
}
*/
#endif /* filtering_hpp */
