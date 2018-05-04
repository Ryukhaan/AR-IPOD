//
//  marching_cube.hpp
//  AR-IPOD
//
//  Created by Remi Decelle on 04/05/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#ifndef marching_cube_hpp
#define marching_cube_hpp

#include <stdio.h>
#include <simd/common.h>
#include <simd/vector_types.h>
#include <simd/vector_make.h>
#include <simd/vector.h>
#include <simd/conversion.h>
#include <simd/matrix.h>
#include <vector>

simd::float3 interpolate(float isolevel, simd::float3 a, simd::float3 b, float alpha, float beta);
std::vector<simd::float3> polygonise(simd::float3 points[8], float values[8], float isolevel, int* edgeTable, int* triTable);

#endif /* marching_cube_hpp */
