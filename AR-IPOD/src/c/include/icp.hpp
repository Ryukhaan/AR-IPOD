//
//  icp.hpp
//  AR-IPOD
//
//  Created by Remi Decelle on 18/05/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#ifndef icp_hpp
#define icp_hpp

#include <stdio.h>
#include <simd/common.h>
#include <simd/vector_types.h>
#include <simd/vector_make.h>
#include <simd/vector.h>
#include <simd/conversion.h>
#include <simd/matrix.h>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

using namespace std;
using namespace pcl;

void fast_icp(const float* previous_points,
              const float* current_points,
              const void* intrinsics,
              void* rotation,
              void* lie_rotation,
              void* translation,
              void* voxels,
              const int dimension,
              const float resolution,
              const int width,
              const int height)
{
    simd_float4x4 K = ((simd_float4x4 *) intrinsics)[0];
    //simd_float3x3 R = ((simd_float3x3 *) rotation)[0];
    simd::float3  W = ((simd_float3 *) lie_rotation)[0];
    simd_float3x3 R = rotation_from_lie(W);
    simd::float3  T = ((simd_float3 *) translation)[0];
    simd_float4x4 Kinv = simd_inverse(K);
    //simd::float3 dimensions = simd_make_float3(dimension[0], dimension[1], dimension[2]);
    //int size = pow(resolution, 3.0);
    //int square = pow(resolution, 2.0);
    // Calculate mass center of last point cloud and current point cloud
    /*
    int previous_count  = 0;
    int current_count   = 0;
    simd::float3 previous_mass_centre   = simd_make_float3(0, 0, 0);
    simd::float3 current_mass_centre    = simd_make_float3(0, 0, 0);
    for (int k=0; k<height*width; k++)
    {
        int i = k / width;
        int j = k % width;
        // Previous points
        float depth = previous_points[k];
        if ( ! std::isnan(depth) && depth >= 1e-6 ) {
            simd::float4 uv = simd_make_float4(depth * i, depth * j, depth, 1);
            simd::float4 local  = simd_mul(Kinv, uv);
            simd::float3 rlocal  = simd_make_float3(local.x, local.y, local.z);
            //simd::float3 previous_point = simd_mul(simd_transpose(R), rlocal - T);
            simd::float3 previous_point = simd_mul(R, rlocal + T);
            previous_count ++;
            previous_mass_centre += previous_point;
        }
        // Current points
        depth = current_points[k];
        if ( ! std::isnan(depth) && depth >= 1e-6 ) {
            simd::float4 uv = simd_make_float4(depth * i, depth * j, depth, 1);
            simd::float4 local  = simd_mul(Kinv, uv);
            simd::float3 rlocal  = simd_make_float3(local.x, local.y, local.z);
            //simd::float3 current_point = simd_mul(simd_transpose(R), rlocal - T);
            simd::float3 current_point = simd_mul(R, rlocal + T);
            current_count ++;
            current_mass_centre += current_point;
        }
    }
    // Divide by number of points seen
    current_mass_centre     = current_mass_centre / (float) current_count;
    previous_mass_centre    = previous_mass_centre / (float) previous_count;
    // Add translation vector between current and previous points to previous transalation
    T  += (previous_mass_centre - current_mass_centre);
    ((simd_float3 *) translation)[0] = T;
     */
    
    simd::float3 offset = 0.5 * dimension * resolution;
    int square = dimension * dimension;
    int size = pow(dimension, 3);
    
    //simd_float3x3 A = simd_diagonal_matrix(simd_make_float3(0,0,0));
    //simd::float3 b  = simd_make_float3(0, 0, 0);
    
    simd::float3 T_old = T;
    simd::float3 T_new = T;
    simd::float3 W_old = W;
    simd::float3 W_new = W;
    
    while (true) {
        //simd_float6x6 A = simd_diagonal_matrix(simd_make_float6(0,0,0,0,0,0));
        //simd_float6 b  = simd_make_float3(0, 0, 0, 0, 0, 0);
        //float A[6][6];
        //float b[6];
        Eigen::Matrix<float, 6, 6> A;
        Eigen::Matrix<float, 6, 1> b;
        
        T_old = T_new;
        W_old = W_new;
        R = rotation_from_lie(W_new);
        
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j ++) {
                float depth = previous_points[ i*width + j];
                if (std::isnan(depth) || depth < 1e-6) continue;
                simd::float4 uv = simd_make_float4(depth * i, depth * j, depth, 1);
                simd::float4 local  = simd_mul(Kinv, uv);
                simd::float3 global = simd_mul(R, simd_make_float3(local.x, local.y, local.z) + T_new);
                simd_int3 V_ijk = global_to_integer(global + offset, resolution);
                int k = hash_code(V_ijk, resolution);
                int dkx = k + square;
                int dky = k + dimension;
                int dkz = k + 1;
                if (k >= size || k < 0) continue;
                if (dkx >= size || dkx < 0) continue;
                if (dky >= size || dky < 0) continue;
                if (dkz >= size || dkz < 0) continue;
                
                // Estiamte Derivative of SDF
                Eigen::Matrix<float, 3, 1> gradient;
                gradient(0) = ((Voxel *) voxels)[dkx].sdf - ((Voxel *) voxels)[k].sdf;
                gradient(1) = ((Voxel *) voxels)[dky].sdf - ((Voxel *) voxels)[k].sdf;
                gradient(2) = ((Voxel *) voxels)[dkz].sdf - ((Voxel *) voxels)[k].sdf;
                
                // Estimate Jacobian
                Eigen::Matrix<float, 6, 1> jacobian;
                //jacobian(0) = - expf(-W_new.x)  * global.z * gradient(1)  + expf(W_new.x)     * global.y * gradient(2);
                //jacobian(1) =   expf(W_new.y)   * global.z * gradient(0)  - expf(-W_new.y)    * global.x * gradient(2);
                //jacobian(2) = - expf(-W_new.z)  * global.y * gradient(0)  + expf(W_new.z)     * global.x * gradient(1);
                jacobian(0) = - global.z * gradient(1)  +  global.y * gradient(2);
                jacobian(1) =   global.z * gradient(0)  -  global.x * gradient(2);
                jacobian(2) = - global.y * gradient(0)  +  global.x * gradient(1);
                jacobian(3) = gradient(0);
                jacobian(4) = gradient(1);
                jacobian(5) = gradient(2);
                A += jacobian * jacobian.transpose();
                b += ((Voxel *) voxels)[k].sdf * jacobian;
            }
        }
        Eigen::Matrix<float, 6, 1> wt = A.inverse() * b;
        if (isnan(wt(0))) break;
        if (isnan(wt(1))) break;
        if (isnan(wt(2))) break;
        if (isnan(wt(3))) break;
        if (isnan(wt(4))) break;
        if (isnan(wt(5))) break;
            
        W_new -= simd_make_float3(wt(0), wt(1), wt(2));
        T_new -= simd_make_float3(wt(3), wt(4), wt(5));
        if (simd_max(simd_norm_inf(T_new - T_old), simd_norm_inf(W_new - W_old)) < 1e-2)
            break;
    }
                                                            
    ((simd_float3x3 *) rotation)[0]     = rotation_from_lie(W_new);
    ((simd_float3 *) translation)[0]    = T_new;
    ((simd_float3 *) lie_rotation)[0]   = W_new;
    
    /*
    PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr cloud_out (new PointCloud<PointXYZ>);
    cloud_in->width    = width * height;
    cloud_in->height   = 1;
    cloud_in->is_dense = false;
    cloud_out->width    = width * height;
    cloud_out->height   = 1;
    cloud_out->is_dense = false;
    cloud_in->points.resize (cloud_in->width * cloud_in->height);
    cloud_out->points.resize (cloud_out->width * cloud_out->height);
    for (int i =0; i<height-1; i++)
    {
        for (int j = 0; j<width-1; j++)
        {
            float z = previous_points[i * width + j];
            if (isnan(z) || z < 1e-6)
            {
                cloud_in->points[i * width + j] = PointXYZ(0,0,0);
                continue;
            }
            simd::float4 uvz = simd_make_float4(z * i, z * j, z, 1);
            simd::float4 local = simd_mul(Kinv, uvz);
            simd::float3 hlocal = simd_make_float3(local.x, local.y, local.z);
            simd::float3 global = simd_mul(R, hlocal) + T;

            cloud_in->points[i * width + j] = PointXYZ(global.x, global.y, global.z);
        }
    }
    for (int i =0; i<height-1; i++)
    {
        for (int j = 0; j<width-1; j++)
        {
            float z = current_points[i * width + j];
            if (isnan(z) || z < 1e-6)
            {
                cloud_out->points[i * width + j] = PointXYZ(0,0,0);
                continue;
            }
            simd::float4 uvz = simd_make_float4(z * i, z * j, z, 1);
            simd::float4 local = simd_mul(Kinv, uvz);
            simd::float3 hlocal = simd_make_float3(local.x, local.y, local.z);
            simd::float3 global = simd_mul(R, hlocal) + T;

            cloud_out->points[i * width + j] = PointXYZ(global.x, global.y, global.z);
        }
    }
    cout << "Start ICP" << endl;
    //IterativeClosestPointWithNormals<PointXYZRGBNormal, PointXYZRGBNormal> icp;
    IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    icp.setMaximumIterations(10);
    icp.setMaxCorrespondenceDistance(1.0);
    //icp.setSearchMethodTarget(search::KdTree<PointXYZRGBNormal>::Ptr (new search::KdTree<PointXYZRGBNormal>));
    icp.setSearchMethodTarget(search::KdTree<PointXYZ>::Ptr (new search::KdTree<PointXYZ>));
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    icp.align(*cloud_in);
    cout << "Has converged:" << icp.hasConverged() << endl;
    //cout << "Score: " << icp.getFitnessScore() << endl;
    auto Rt = icp.getFinalTransformation();
    cout << Rt << endl;
    ((simd_float3x3 *) rotation)[0] = simd_matrix_from_rows(
                                                            simd_make_float3(Rt(0,0), Rt(0,1), Rt(0,2)),
                                                            simd_make_float3(Rt(1,0), Rt(1,1), Rt(1,2)),
                                                            simd_make_float3(Rt(2,0), Rt(2,1), Rt(2,2))
                                                            );
    ((simd_float3 *) translation)[0] = simd_make_float3(Rt(0,3), Rt(1,3), Rt(1,3));
    */
}
#endif /* icp_hpp */
