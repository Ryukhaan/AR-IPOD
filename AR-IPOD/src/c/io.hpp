//
//  io.hpp
//  AR-IPOD
//
//  Created by Remi Decelle on 04/05/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#ifndef io_hpp
#define io_hpp

#include <stdio.h>
#include <simd/common.h>
#include <simd/vector_types.h>
#include <simd/vector_make.h>
#include <simd/vector.h>

void save_meshing_ply_format(const simd::float3* points, const char* file_name, int number_of_triangles);

#endif /* io_hpp */
