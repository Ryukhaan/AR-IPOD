//
//  icp.hpp
//  AR-IPOD
//
//  Created by Remi Decelle on 18/05/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

#ifndef icp_hpp
#define icp_hpp

#include "super4pcs.h"
#include "logger.h"
#include "constants.h"
#include <Eigen/Core>
//#include <Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>


#include <stdio.h>
#include <simd/common.h>
#include <simd/vector_types.h>
#include <simd/vector_make.h>
#include <simd/vector.h>
#include <simd/conversion.h>
#include <simd/matrix.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/ia_kfpcs.h>
#include <pcl/registration/gicp.h>

#include <pcl/features/normal_3d.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace std;
using namespace GlobalRegistration;

// Typedefs, convenience.
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


void super4PCS(const float* previous,
               const float* current,
               const void* voxels,
               const int width,
               const int height,
               void* rotation,
               void* translation,
               const void* intrinsics,
               const float resolution,
               const int dimension)
{
    simd_float4x4 K = ((simd_float4x4 *) intrinsics)[0];
    simd_float4x4 Kinv = simd_inverse(K);
    simd_float3x3 R = ((simd_float3x3 *) rotation)[0];
    simd::float3  T = ((simd_float3 *) translation)[0];
    
    Eigen::Matrix<Point3D::Scalar, 4, 4> Rt;
    Rt << R.columns[0].x   , R.columns[1].x   , R.columns[2].x, T.x  ,
    R.columns[0].y     , R.columns[1].y   , R.columns[2].y, T.y  ,
    R.columns[0].z     , R.columns[1].z   , R.columns[2].z, T.z  ,
    0                  , 0                , 0             , 1    ;
    vector<Point3D> P, Q;
    simd::float4 uvz, local;
    simd::float3 global;
    float z;
    //float cy = 1; float cx = 1;
    float cy = 6; float cx = 6;
    
    Match4PCSOptions options = Match4PCSOptions();
    Utils::Logger log = Utils::Logger();
    MatchSuper4PCS algo4PCS = MatchSuper4PCS(options, log);
    
    int squared = dimension * dimension;
    float global_offset = 0.5 * dimension * resolution;
    for (int i=0; i < dimension; i++)
        for (int j=0; j<dimension; j++)
            for (int k=0; k<dimension; k++)
            {
                int n   = i * squared + j * dimension + k;
                simd_int3   v_ijk = simd_make_int3(i, j, k);
                //simd::float3 centroid = create_centroid(n, resolutions.x, dimension) - offset;
                simd::float3 centroid = integer_to_global(v_ijk, resolution) - global_offset;
                if ( fabs(((Voxel *) voxels)[n].sdf) <= 1e-2)
                    P.push_back(Point3D(centroid.x, centroid.y, centroid.z));
            }
    for (int i = 0; i<height; i++)
    {
        for (int j = 0; j<height; j++)
        {
            /*
             z       = previous[i * width + j];
             if (z > 1e-6)
             {
             uvz     = simd_make_float4(z * i * cy, z * j * cx, z, 1);
             local   = simd_mul(Kinv, uvz);
             global  = simd_mul(R, simd_make_float3(local.x, local.y, local.z)) + T;
             P.push_back(Point3D(global.x, global.y, global.z));
             }
             */
            z       = current[i * width + j];
            if (z > 1e-6)
            {
                uvz     = simd_make_float4(z * i * coy, z * j * cox, z, 1);
                local   = simd_mul(Kinv, uvz);
                global  = simd_mul(R, simd_make_float3(local.x, local.y, local.z)) + T;
                Q.push_back(Point3D(global.x, global.y, global.z));
            }
        }
    }
    //algo4PCS.Initialize(P, Q);
    algo4PCS.ComputeTransformation(P, &Q, Rt);
    ((simd_float3 *) translation)[0]    = T + simd_make_float3(Rt(0, 3), Rt(1, 3), Rt(2, 3));
    ((simd_float3x3 *) rotation)[0]     = simd_mul(simd_matrix_from_rows(
                                                                         simd_make_float3(Rt(0,0), Rt(0,1), Rt(0,2)),
                                                                         simd_make_float3(Rt(1,0), Rt(1,1), Rt(1,2)),
                                                                         simd_make_float3(Rt(2,0), Rt(2,1), Rt(2,2))
                                                                         ),
                                                   R);
}

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

Eigen::Matrix3d exponential_map(Eigen::Vector3d omega) {
    Eigen::Matrix3d R;
    R <<0, omega(2), -omega(1),
        -omega(2), 0, omega(0),
        omega(1), -omega(0), 0;
    return R.exp();
}

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
    Eigen::VectorXd SDF_derivative(6), zeros(6);
    SDF_derivative << 0, 0, 0, 0, 0, 0;
    zeros = SDF_derivative;
    Eigen::Vector3d tmp             = Eigen::Vector3d::Zero();
    //simd::float3    current_world_point;
    simd::int3      current_voxel_point;
    
    float offset = dimension * resolution * 0.5;
    int   max_size = pow(dimension, 3.0);
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
    current_voxel_point = global_to_integer(current_world_point + offset, resolution);
    if (current_voxel_point.x < 0
        || current_voxel_point.y < 0
        || current_voxel_point.z < 0)
        return zeros;
    if (current_voxel_point.x >= max_size
        || current_voxel_point.y >= max_size
        || current_voxel_point.z >= max_size)
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
    plus_h_voxel_point  = global_to_integer(simd_make_float3(tmp(0), tmp(1), tmp(2)) + offset, resolution);
    plus_h_sdf_value    = interpolate_distance(voxels, plus_h_voxel_point, dimension);
    if (plus_h_sdf_value == 0.0)    return zeros;
    
    tmp = r1m * camera_point +  T;
    minus_voxel_point   = global_to_integer(simd_make_float3(tmp(0), tmp(1), tmp(2)) + offset, resolution);
    minus_h_sdf_value   = interpolate_distance(voxels, minus_voxel_point, dimension);
    if (minus_h_sdf_value == 0.0)   return zeros;
    
    SDF_derivative(3) = (plus_h_sdf_value - minus_h_sdf_value) / (2 * (w_h));
    
    //wy derivative
    tmp  = r2p * camera_point +  T;
    plus_h_voxel_point  = global_to_integer(simd_make_float3(tmp(0), tmp(1), tmp(2)) + offset, resolution);
    plus_h_sdf_value    = interpolate_distance(voxels, plus_h_voxel_point, dimension);
    if (plus_h_sdf_value == 0.0)    return zeros;
    
    tmp = r2m * camera_point +  T;
    minus_voxel_point   = global_to_integer(simd_make_float3(tmp(0), tmp(1), tmp(2)) + offset, resolution);
    minus_h_sdf_value   = interpolate_distance(voxels, minus_voxel_point, dimension);
    if (minus_h_sdf_value == 0.0)   return zeros;
    
    SDF_derivative(4) = (plus_h_sdf_value - minus_h_sdf_value) / (2 * (w_h));
    
    //wz derivative
    tmp = r3p * camera_point +  T;
    plus_h_voxel_point  = global_to_integer(simd_make_float3(tmp(0), tmp(1), tmp(2)) + offset, resolution);
    plus_h_sdf_value    = interpolate_distance(voxels, plus_h_voxel_point, dimension);
    if (plus_h_sdf_value == 0.0)    return zeros;
    
    tmp = r3m * camera_point +  T;
    minus_voxel_point   = global_to_integer(simd_make_float3(tmp(0), tmp(1), tmp(2)) + offset, resolution);
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
    
    Eigen::Matrix3d R;
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
    
    /*
    simd::float3 offset = 0.5 * dimension * resolution;
    int square = dimension * dimension;
    int size = pow(dimension, 3);
    */
    int n_iter      = 0;
    int max_iter    = 15;
    float maximum_twist_diff = 1e-3;
    float v_h   = 1.0;
    float w_h   = 0.01;
    float v_h2  = 2*v_h;
    float w_h2  = 2*w_h;
    float v_h2_width    = v_h2 * resolution;
    float v_h2_height   = v_h2 * resolution;
    float v_h2_depth    = v_h2 * resolution;
    
    //twist_diff = (v1,v2,v3,w1,w2,w3)
    Eigen::VectorXd twist_diff = Eigen::VectorXd::Zero(6);
    Eigen::Matrix3d R_diff;
    Eigen::Matrix3d r1p, r1m, r2p, r2m, r3p, r3m;
    while (true)
    {
        if (n_iter > max_iter) break;
        n_iter ++;
        
        Eigen::VectorXd  old_twist = twist_diff;
        Eigen::MatrixXd  A(6,6); //= Eigen::MatrixXd::Zero(6);
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
                if (point.x == 0 && point.y == 0 && point.z == 0) continue;
                
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
        twist_diff -= A.inverse() * b;
        
        R = exponential_map(Eigen::Vector3d(twist_diff(3), twist_diff(4), twist_diff(5)));
        T = Eigen::Vector3d(twist_diff(0), twist_diff(1), twist_diff(2));
        //Eigen::Affine3d aff = direct_exponential_map(twist_diff,1.0);

        if (twist_diff(0, 0) < maximum_twist_diff
            && twist_diff(1, 0) < maximum_twist_diff
            && twist_diff(2, 0) < maximum_twist_diff
            && twist_diff(3, 0) < maximum_twist_diff
            && twist_diff(4, 0) < maximum_twist_diff
            && twist_diff(5, 0) < maximum_twist_diff) {
            cout << "STOP Gauss Newton at step: "<< n_iter << endl;
            break;
        }
        
        if ( (twist_diff - old_twist).lpNorm<Eigen::Infinity>() < 1e-5 ) break;
        
        //reorthomolize rotation
        //R = R - r_new.transpose() * R;
        //T = T - r_new.transpose() * t_new;

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
    */
    R = exponential_map(Eigen::Vector3d(twist_diff(3), twist_diff(4), twist_diff(5)));
    T = Eigen::Vector3d(twist_diff(0), twist_diff(1), twist_diff(2));
    ((simd_float3x3 *) rotation)[0] = simd_matrix_from_rows(
                                                            simd_make_float3(R(0,0), R(0,1), R(0,2)),
                                                            simd_make_float3(R(1,0), R(1,1), R(1,2)),
                                                            simd_make_float3(R(2,0), R(2,1), R(2,2))
                                                            );
    ((simd_float3 *) translation)[0] = simd_make_float3(T(0), T(1), T(2));
}

void icp(const float* previous,
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
         const int max_num_iter)
{
    // Init all matrices : R, K and T
    simd_float4x4 K = ((simd_float4x4 *) intrinsics)[0];
    simd_float4x4 Kinv = simd_inverse(K);
    simd_float3x3 R = ((simd_float3x3 *) rotation)[0];
    simd::float3  T = ((simd_float3 *) translation)[0];
    
    // Init camera position : Rt = old one and Rtprime = new one
    Eigen::Matrix<Point3D::Scalar, 4, 4> Rt, Rtprime = Eigen::Matrix4f::Identity ();
    Rt <<   R.columns[0].x  , R.columns[1].x   , R.columns[2].x, T.x  ,
            R.columns[0].y  , R.columns[1].y   , R.columns[2].y, T.y  ,
            R.columns[0].z  , R.columns[1].z   , R.columns[2].z, T.z  ,
            0               , 0                , 0             , 1    ;
    
    simd::float4 uvz, local;
    simd::float3 global;
    float z;
    console::TicToc clock = console::TicToc();
    
    PointCloud<PointNormalT>::Ptr cloud_in (new PointCloud<PointNormalT>);
    PointCloud<PointNormalT>::Ptr cloud_out (new PointCloud<PointNormalT>);

    // Build previous global vertex map
    int sh = height / 2 - 100;
    int eh = height / 2 + 100;
    int sw = width / 2 - 60;
    int ew = width / 2 + 60;
    for (int i = sh; i<eh; i++)
    {
        for (int j = sw; j<ew; j++)
        {
            // Neighbors indexes
            int x = i * width + j;
            int rn = i * width + j + 1;
            int dn = (i+1) * width + j;
            
            // Neighbors depths
            z       = previous[x];
            float zr = previous[rn];
            float zd = previous[dn];
            
            // Append PointNormalT to Point Cloud In
            if (z > 1e-6 && zr > 1e-6 && zd > 1e-6 && z < thresh_depth)
            {
                uvz     = simd_make_float4(z * i * coy, z * j * cox, z, 1);
                local   = simd_mul(Kinv, uvz);
                global  = simd_mul(R, simd_make_float3(local.x, local.y, local.z)) + T;
                // Compute normal
                simd::float4 uvz_right   = simd_make_float4(zr * i * coy, zr * j * cox, zr, 1);
                simd::float4 local_right = simd_mul(Kinv, uvz_right);
                simd::float4 uvz_down    = simd_make_float4(zd * i * coy, zd * j * cox, zd, 1);
                simd::float4 local_down  = simd_mul(Kinv, uvz_down);
                simd::float4 normal = simd_dot((local_right - local), (local_down - local));
                normal = normal / simd_length(normal);
                //P.push_back(Point3D(global.x, global.y, global.z));
                PointNormalT p;
                p.x = global.x;
                p.y = global.y;
                p.z = global.z;
                p.normal_x = normal.x;
                p.normal_y = normal.y;
                p.normal_z = normal.z;
    
                cloud_in->push_back(p);
                
            }
            
            // Same for current depth map
            z       = current[x];
            zr = current[rn];
            zd = current[dn];
            if (z > 1e-6 && zr > 1e-6 && zd > 1e-6 && z < thresh_depth)
            {
                uvz     = simd_make_float4(z * i * coy, z * j * cox, z, 1);
                local   = simd_mul(Kinv, uvz);
                global  = simd_mul(R, simd_make_float3(local.x, local.y, local.z)) + T;
                // Compute normal
                simd::float4 uvz_right   = simd_make_float4(zr * i * coy, zr * j * cox, zr, 1);
                simd::float4 local_right = simd_mul(Kinv, uvz_right);
                simd::float4 uvz_down    = simd_make_float4(zd * i * coy, zd * j * cox, zd, 1);
                simd::float4 local_down  = simd_mul(Kinv, uvz_down);
                simd::float4 normal = simd_dot((local_right - local), (local_down - local));
                normal = normal / simd_length(normal);
                //Q.push_back(Point3D(global.x, global.y, global.z));
                PointNormalT p;
                p.x = global.x;
                p.y = global.y;
                p.z = global.z;
                p.normal_x = normal.x;
                p.normal_y = normal.y;
                p.normal_z = normal.z;
                
                cloud_out->push_back(p);
                
            }
        }
    }
    cloud_in->width     = cloud_in->points.size();
    cloud_in->height    = 1;
    cloud_in->is_dense  = false;
    cloud_out->width    = cloud_out->points.size();
    cloud_out->height   = 1;
    cloud_out->is_dense = false;
    cloud_in->points.resize (cloud_in->width * cloud_in->height);
    cloud_out->points.resize (cloud_out->width * cloud_out->height);
    
    clock.tic();
    // Compute surface normals and curvature
    PointCloud<PointNormalT>::Ptr points_with_normals_src (new PointCloud<PointNormalT>);
    PointCloud<PointNormalT>::Ptr points_with_normals_tgt (new PointCloud<PointNormalT>);
    
    NormalEstimation<PointNormalT, PointNormalT> norm_est;
    search::KdTree<PointNormalT>::Ptr tree (new search::KdTree<PointNormalT> ());
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (4);
    
    norm_est.setInputCloud (cloud_out);
    norm_est.compute (*points_with_normals_src);
    copyPointCloud (*cloud_out, *points_with_normals_src);
    
    norm_est.setInputCloud (cloud_in);
    norm_est.compute (*points_with_normals_tgt);
    copyPointCloud (*cloud_in, *points_with_normals_tgt);
    
    IterativeClosestPoint<PointNormalT, PointNormalT> icp;
    icp.setMaxCorrespondenceDistance (0.1);
    icp.setMaximumIterations (1);
    
    icp.setInputSource(points_with_normals_src);
    icp.setInputTarget(points_with_normals_tgt);
    PointCloud<PointNormalT> cloud_aligned;
    icp.align(cloud_aligned);
    
    /*
     float fitness = 65536.0;
     float proba = 1.0;
     int max_iteration = 1, turn = 0;
     clock.tic();
     while (true)
     {
     if (turn > max_iteration) break;
     // Choose new points
     PointCloud<PointNormalT>::Ptr sub_points_src (new PointCloud<PointNormalT>);
     PointCloud<PointNormalT>::Ptr sub_points_tgt (new PointCloud<PointNormalT>);
     for (PointNormalT p : cloud_in->points) {
     float x = (rand() % 1000) / 1000.0;
     if ( x <= proba)
     sub_points_tgt->push_back(p);
     }
     for (PointNormalT p : cloud_out->points) {
     float x = (rand() % 1000) / 1000.0;
     if ( x <= proba)
     sub_points_src->push_back(p);
     }
     sub_points_src->width    = sub_points_src->points.size();
     sub_points_src->height   = 1;
     sub_points_src->is_dense = false;
     sub_points_tgt->width    = sub_points_tgt->points.size();
     sub_points_tgt->height   = 1;
     sub_points_tgt->is_dense = false;
     
     sub_points_src->points.resize (sub_points_src->width * sub_points_src->height);
     sub_points_tgt->points.resize (sub_points_tgt->width * sub_points_tgt->height);
     
     icp.setInputSource(sub_points_src);
     icp.setInputTarget(sub_points_tgt);
     PointCloud<PointNormalT> cloud_aligned;
     icp.align(cloud_aligned);
     
     if (icp.getFitnessScore() < fitness)
     {
     fitness = icp.getFitnessScore();
     Rtprime = icp.getFinalTransformation();
     }
     turn++;
     }
     */
    cout << clock.toc() << endl;
    
    // Get the transformation that aligned cloud_in to cloud_out
    Rt = icp.getFinalTransformation() * Rt;
    cout << Rt << endl;
    
    ((simd_float3 *) translation)[0]   = simd_make_float3(Rt(0, 3), Rt(1, 3), Rt(2, 3));
    ((simd_float3x3 *) rotation)[0]    = simd_matrix_from_rows(
                                                               simd_make_float3(Rt(0,0), Rt(0,1), Rt(0,2)),
                                                               simd_make_float3(Rt(1,0), Rt(1,1), Rt(1,2)),
                                                               simd_make_float3(Rt(2,0), Rt(2,1), Rt(2,2))
                                                               );
}

#endif /* icp_hpp */
