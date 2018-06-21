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

#include <Eigen/Core>
//#include <Eigen/Eigen>
//#include <unsupported/Eigen/MatrixFunctions>

using namespace std;

/*
simd::float3x3 UThetaToAffine3d(const simd::float3 u)
{
    //Eigen::Affine3d rd;
    simd::float3x3 rd;
    double theta, si, co, sinc, mcosc;
    theta   = simd_length(u);
    //theta   = sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
    si      = sin(theta);
    co      = cos(theta);
    sinc    = f_sinc(si,theta);
    mcosc   = f_mcosc(co,theta);
    
    rd.columns[0][0] = co + mcosc * u.x * u.x;
    rd.columns[0][1] = -sinc * u.z   + mcosc * u.x * u.y;
    rd.columns[0][2] = sinc * u.y    + mcosc * u.x * u.z;
    rd.columns[1][0] = sinc * u.z    + mcosc * u.y * u.x;
    rd.columns[1][1] = co + mcosc*u[1]*u[1];
    rd.columns[1][2] = -sinc * u.x   + mcosc * u.y * u.z;
    rd.columns[2][0] = -sinc * u.y   + mcosc * u.z * u.x;
    rd.columns[2][1] = sinc * u.x    + mcosc * u.z * u.y;
    rd.columns[2][2] = co + mcosc*u[2]*u[2];
    
    return simd_transpose(rd);
}

Eigen::Affine3d direct_exponential_map(const Eigen::VectorXd &v, double delta_t)
{
    double theta,si,co,sinc,mcosc,msinc;
    //Eigen::Vector3d u;
    //Eigen::Affine3d rd;
    //rd.setIdentity();
    //Eigen::Vector3d dt;
    //Eigen::VectorXd v_dt = v * delta_t;
    simd::float3 u;
    simd::float3x3 rd;
    simd::float3 dt;
    simd::float3 v_dt = delta_t * v;
    
    u.x = v_dt[3];
    u.y = v_dt[4];
    u.z = v_dt[5];
    
    rd = UThetaToAffine3d(u);
    
    theta = sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
    si = sin(theta);
    co = cos(theta);
    sinc = f_sinc(si,theta);
    mcosc = f_mcosc(co,theta);
    msinc = f_msinc(si,theta);
    
    dt[0] = v_dt[0] * (sinc + u[0]*u[0]*msinc)
    + v_dt[1]*(u[0]*u[1]*msinc - u[2]*mcosc)
    + v_dt[2]*(u[0]*u[2]*msinc + u[1]*mcosc);
    
    dt[1] = v_dt[0] * (u[0]*u[1]*msinc + u[2]*mcosc)
    + v_dt[1]*(sinc + u[1]*u[1]*msinc)
    + v_dt[2]*(u[1]*u[2]*msinc - u[0]*mcosc);
    
    dt[2] = v_dt[0] * (u[0]*u[2]*msinc - u[1]*mcosc)
    + v_dt[1]*(u[1]*u[2]*msinc + u[0]*mcosc)
    + v_dt[2]*(sinc + u[2]*u[2]*msinc);
    
    Eigen::Affine3d Delta;
    Delta.setIdentity();
    Delta = rd;
    Delta(0,3) = dt[0];
    Delta(1,3) = dt[1];
    Delta(2,3) = dt[2];
    
    return Delta;
}
*/

Eigen::VectorXd get_partial_derivative(const Voxel* voxels,
                                                   const Eigen::Vector3d camera_point,
                                                   const simd::float3 current_world_point,
                                                   const float resolution,
                                                   const int dimension,
                                                   const Eigen::Vector3d T,
                                                   const Eigen::Matrix3d r1p,
                                                   const Eigen::Matrix3d r1m,
                                                   const Eigen::Matrix3d r2p,
                                                   const Eigen::Matrix3d r2m,
                                                   const Eigen::Matrix3d r3p,
                                                   const Eigen::Matrix3d r3m,
                                                   double& sdf_val) {
    Eigen::VectorXd SDF_derivative, zeros;
    SDF_derivative << 0, 0, 0, 0, 0, 0;
    zeros = SDF_derivative;
    Eigen::Vector3d tmp             = Eigen::Vector3d::Zero();
    //simd::float3    current_world_point;
    simd::int3      current_voxel_point;
    
    float v_h   = 1.0;
    float w_h   = 0.01;
    float v_h2  = 2*v_h;
    //float w_h2  = 2*w_h;
    float v_h2_width    = v_h2 * resolution;
    float v_h2_height   = v_h2 * resolution;
    float v_h2_depth    = v_h2 * resolution;
    
    //we use central difference
    simd::int3 plus_h_voxel_point;
    simd::int3 minus_voxel_point;
    float plus_h_sdf_value;
    float minus_h_sdf_value;
    //this->project_camera_to_world(camera_point, current_world_point);
    current_voxel_point = global_to_integer(current_world_point, resolution);
    if (current_voxel_point.x < 0
        || current_voxel_point.y < 0
        || current_voxel_point.z < 0)
        return zeros;
    if (current_voxel_point.x >= dimension
        || current_voxel_point.y >= dimension
        || current_voxel_point.z >= dimension)
        return zeros;
    
    sdf_val = interpolate_distance(voxels, current_voxel_point, dimension);
    if (sdf_val == 0.0) return zeros;
    
    //tx derivative
    plus_h_voxel_point      = current_voxel_point;
    plus_h_voxel_point.x    += v_h;
    minus_voxel_point       = current_voxel_point;
    minus_voxel_point.x     -= v_h;
    plus_h_sdf_value = interpolate_distance(voxels, plus_h_voxel_point, dimension);
    if (plus_h_sdf_value == 0.0)    return zeros;
    minus_h_sdf_value = interpolate_distance(voxels, minus_voxel_point, dimension);
    if (minus_h_sdf_value == 0.0)   return zeros;
    SDF_derivative(0) = (plus_h_sdf_value - minus_h_sdf_value) / v_h2_width;
    
    //ty derivative
    plus_h_voxel_point = current_voxel_point;
    plus_h_voxel_point.y += v_h;
    minus_voxel_point = current_voxel_point;
    minus_voxel_point.y -= v_h;
    plus_h_sdf_value = interpolate_distance(voxels, plus_h_voxel_point, dimension);
    if (plus_h_sdf_value == 0.0)    return zeros;
    minus_h_sdf_value = interpolate_distance(voxels,minus_voxel_point, dimension);
    if (minus_h_sdf_value == 0.0)   return zeros;
    SDF_derivative(1) = (plus_h_sdf_value - minus_h_sdf_value) / v_h2_height;
    
    //tz derivative
    plus_h_voxel_point = current_voxel_point;
    plus_h_voxel_point.z += v_h;
    minus_voxel_point = current_voxel_point;
    minus_voxel_point.z -= v_h;
    plus_h_sdf_value = interpolate_distance(voxels, plus_h_voxel_point, dimension);
    if (plus_h_sdf_value == 0.0)    return zeros;
    minus_h_sdf_value = interpolate_distance(voxels, minus_voxel_point, dimension);
    if (minus_h_sdf_value == 0.0)   return zeros;
    SDF_derivative(2) = (plus_h_sdf_value - minus_h_sdf_value) / v_h2_depth;
    
    //wx derivative
    tmp  = r1p * camera_point +  T;
    plus_h_voxel_point  = global_to_integer(simd_make_float3(tmp(0), tmp(1), tmp(2)), resolution);
    plus_h_sdf_value    = interpolate_distance(voxels, plus_h_voxel_point, dimension);
    if (plus_h_sdf_value == 0.0)    return zeros;
    
    tmp = r1m * camera_point +  T;
    minus_voxel_point   = global_to_integer(simd_make_float3(tmp(0), tmp(1), tmp(2)), resolution);
    minus_h_sdf_value   = interpolate_distance(voxels, minus_voxel_point, dimension);
    if (minus_h_sdf_value == 0.0)   return zeros;
    
    SDF_derivative(3) = (plus_h_sdf_value - minus_h_sdf_value) / (2 * (w_h));
    
    //wy derivative
    tmp  = r2p * camera_point +  T;
    plus_h_voxel_point  = global_to_integer(simd_make_float3(tmp(0), tmp(1), tmp(2)), resolution);
    plus_h_sdf_value    = interpolate_distance(voxels, plus_h_voxel_point, dimension);
    if (plus_h_sdf_value == 0.0)    return zeros;
    
    tmp = r2m * camera_point +  T;
    minus_voxel_point   = global_to_integer(simd_make_float3(tmp(0), tmp(1), tmp(2)), resolution);
    minus_h_sdf_value   = interpolate_distance(voxels, minus_voxel_point, dimension);
    if (minus_h_sdf_value == 0.0)   return zeros;
    
    SDF_derivative(4) = (plus_h_sdf_value - minus_h_sdf_value) / (2 * (w_h));
    
    //wz derivative
    tmp = r3p * camera_point +  T;
    plus_h_voxel_point  = global_to_integer(simd_make_float3(tmp(0), tmp(1), tmp(2)), resolution);
    plus_h_sdf_value    = interpolate_distance(voxels, plus_h_voxel_point, dimension);
    if (plus_h_sdf_value == 0.0)    return zeros;
    
    tmp = r3m * camera_point +  T;
    minus_voxel_point   = global_to_integer(simd_make_float3(tmp(0), tmp(1), tmp(2)), resolution);
    minus_h_sdf_value   = interpolate_distance(voxels, minus_voxel_point, dimension);
    if (minus_h_sdf_value == 0.0)   return zeros;
    
    SDF_derivative(5) = (plus_h_sdf_value - minus_h_sdf_value) / (2 * (w_h));
    
    return SDF_derivative;
}

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
    simd_float3x3 R_tmp = ((simd_float3x3 *) rotation)[0];
    //simd::float3  W = ((simd_float3 *) lie_rotation)[0];
    //simd_float3x3 R_tmp = rotation_from_lie(W);
    simd::float3  T_tmp = ((simd_float3 *) translation)[0];
    simd_float4x4 Kinv = simd_inverse(K);
    
    Eigen::MatrixXd R;
    Eigen::Vector3d T;
    R <<    R_tmp.columns[0].x, R_tmp.columns[1].x, R_tmp.columns[2].x,
            R_tmp.columns[0].y, R_tmp.columns[1].y, R_tmp.columns[2].y,
            R_tmp.columns[0].z, R_tmp.columns[1].z, R_tmp.columns[2].z;
    T << T_tmp.x, T_tmp.y, T_tmp.z;
    
    
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
    
    int n_iter      = 0;
    int max_iter    = 20;
    float maximum_twist_diff = 1e-3;
    float v_h   = 1.0;
    float w_h   = 0.01;
    float v_h2  = 2*v_h;
    float w_h2  = 2*w_h;
    float v_h2_width    = v_h2 * resolution;
    float v_h2_height   = v_h2 * resolution;
    float v_h2_depth    = v_h2 * resolution;
    
    Eigen::VectorXd twist_diff = Eigen::VectorXd::Zero(6);
    Eigen::Matrix3d R_diff;
    Eigen::Matrix3d r1p, r1m, r2p, r2m, r3p, r3m;
    while (true)
    {
        if (n_iter > max_iter) break;
        n_iter ++;
        
        Eigen::MatrixXd  A; //= Eigen::MatrixXd::Zero(6);
        Eigen::VectorXd  b = Eigen::VectorXd::Zero(6);
        /*
         * We want to calculate the partial derivative of our sdf
         * with respekt to the twist coordinates. So we build up
         * the linearized version of R' = w*R => R+_w = (I+w) R
         *
         * R_w1+ = 1.0 0.0 0.0
         *         0.0 1.0 -w1
         *         0.0  w1 1.0
         */
        R_diff(0, 0) = 1.0;
        R_diff(0, 1) = 0.0;
        R_diff(0, 2) = 0.0;
        R_diff(1, 0) = 0.0;
        R_diff(1, 1) = 1.0;
        R_diff(1, 2) = -w_h;
        R_diff(2, 0) = 0.0;
        R_diff(2, 1) = w_h;
        R_diff(2, 2) = 1.0;
        r1p = R_diff * R;

        R_diff(1, 2) = w_h;
        R_diff(2, 1) = -w_h;
        r1m = R_diff * R;

        R_diff(1, 2) = 0;
        R_diff(2, 1) = 0;
        R_diff(0, 2) = w_h;
        R_diff(2, 0) = -w_h;
        r2p = R_diff * R;

        R_diff(0, 2) = -w_h;
        R_diff(2, 0) = w_h;
        r2m = R_diff * R;

        R_diff(0, 2) = 0;
        R_diff(2, 0) = 0;
        R_diff(0, 1) = -w_h;
        R_diff(1, 0) = w_h;
        r3p = R_diff * R;

        R_diff(0, 1) = w_h;
        R_diff(1, 0) = -w_h;
        r3m = R_diff * R;
/*
#pragma omp parallel
       {
        boost::shared_ptr<Eigen::Matrix<double, 6, 6> > A_ptr;
        boost::shared_ptr<Eigen::VectorXd > B_ptr;
        A_ptr = boost::make_shared<Eigen::Matrix<double, 6, 6> >();
        B_ptr = boost::make_shared<Eigen::VectorXd >();
        A_ptr->setZero();
        B_ptr->setZero();
        A_array[omp_get_thread_num()] = A_ptr;
        B_array[omp_get_thread_num()] = B_ptr;
*/
        bool is_interpolated = false;
        Eigen::VectorXd SDF_derivative;
        double int_dist = 0;
        Eigen::Vector3d camera_point = Eigen::Vector3d::Zero();
        //#pragma omp  for
        //iterate all image points
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                
                float z = current_points[i*width + j];
                simd::float4 point = simd_mul(Kinv, simd_make_float4( z * i, z * j, z, 1.0));
                //does a good depth exist?
                if (isnan(point.x) || isnan(point.y) || isnan(point.z)) continue;
                
                camera_point(0) = point.x;
                camera_point(1) = point.y;
                camera_point(2) = point.z;
                
                simd::float3 world_point = simd_mul(R_tmp, simd_make_float3(point.x, point.y, point.z)) + T_tmp;
                //get SDF_derivative
                SDF_derivative  = get_partial_derivative((Voxel*) voxels,
                                                          camera_point, world_point,
                                                          resolution, dimension,
                                                          T, r1p, r1m, r2p, r2m, r3p, r3m,
                                                          int_dist);
                
                //we could calculate SDF_derivative at this point
                if (SDF_derivative == Eigen::VectorXd::Zero(6)) continue;
                A = A + (SDF_derivative * SDF_derivative.transpose());
                b = b + (int_dist * SDF_derivative);
            }
        }
     //}

        //calculate our optimized gradient
        //twist_diff = A.inverse() * b;
        
        //Eigen::Affine3d aff = direct_exponential_map(twist_diff,1.0);
        Eigen::Matrix3d r_new;
        Eigen::Vector3d t_new;
        if (twist_diff(0, 0) < maximum_twist_diff
            && twist_diff(1, 0) < maximum_twist_diff
            && twist_diff(2, 0) < maximum_twist_diff
            && twist_diff(3, 0) < maximum_twist_diff
            && twist_diff(4, 0) < maximum_twist_diff
            && twist_diff(5, 0) < maximum_twist_diff) {
            cout << "STOP Gauss Newton at step: "<< n_iter << endl;
            break;
        }
        
        //reorthomolize rotation

        R = R - r_new.transpose() * R;
        T = T - r_new.transpose() * t_new;
        //this->set_camera_transformation(R, T);
    }
     
    //simd_float3x3 A = simd_diagonal_matrix(simd_make_float3(0,0,0));
    //simd::float3 b  = simd_make_float3(0, 0, 0);
    
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
