//
//  types.h
//  AR-IPOD
//
//  Created by Remi Decelle on 21/06/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#ifndef types_h
#define types_h

#include <Eigen/Dense>
#include <simd/common.h>
#include <simd/vector_types.h>
#include <simd/vector_make.h>
#include <simd/vector.h>
#include <simd/conversion.h>
#include <simd/matrix.h>

typedef Eigen::Matrix<float, 6, 6> Matrix_6x6;
typedef Eigen::Matrix<float, 4, 4> Matrix_4x4;
typedef Eigen::Matrix<float, 3, 3> Matrix_3x3;

typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 4, 1> Vector4f;
typedef Eigen::Matrix<float, 3, 1> Vector3f;
typedef Eigen::Matrix<int, 3, 1>   Vector6i;

typedef Eigen::Matrix<float, 2, 1> Vector2f;
typedef Eigen::Matrix<int, 2, 1> Vector2i;


#endif /* types_h */
