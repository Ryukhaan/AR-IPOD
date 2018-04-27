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


float integerToCenter(float point, int dim, float resolution);
void bridge_initializeCentroids(void* centroids, int size, float resolution);

#endif /* volumeInit_hpp */
