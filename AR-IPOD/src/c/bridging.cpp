//
//  volumeInit.cpp
//  AR-IPOD
//
//  Created by Remi Decelle on 27/04/2018.
//  Copyright © 2018 Remi Decelle. All rights reserved.
//

extern "C" {
    #include "bridging.hpp"
}

#include <simd/common.h>
#include <simd/vector_types.h>
#include <simd/vector_make.h>
#include <simd/vector.h>
#include <simd/conversion.h>
#include <simd/matrix.h>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <limits>
#include <fstream>
//#include <omp.h>

#include "linear_algebra.hpp"
#include "marching_cube.hpp"
#include "io.hpp"
#include "TSDF.hpp"
#include "filtering.hpp"

using namespace std;
                                          
/*
 * GLOBAL FUNCTION (hpp)
 */
void bridge_initializeCentroids(void* centroids, int size, float resolution) {
    //int this_thread = omp_get_thread_num();
    //int num_threads = omp_get_num_threads();
    int count = pow(size, 3.0);
    int square = pow(size, 2.0);
    float offset = (resolution * 0.5);
#pragma omp parallel for shared(centroids) num_threads(num_threads)
    for(int i = 0; i<count; i++) {
        int x = i / square;
        int remainder = i % square;
        int y = remainder / size;
        int z = (float)(((int) remainder) % size);
        //((simd::float3*) centroids)[i][0] = integer_to_global(x, size, resolution) - offset;
        //((simd::float3*) centroids)[i][1] = integer_to_global(y, size, resolution) - offset;
        //((simd::float3*) centroids)[i][2] = integer_to_global(z, size, resolution) - offset;
    }
}

unsigned long bridge_extractMesh(void* triangles,
                                 void* voxels,
                                 const int edgeTable[256],
                                 const int triTable[4096],
                                 const int dimension,
                                 const float isolevel,
                                 const float resolution[3]) {
    unsigned long index = 0;
    int n = dimension;
    simd::float3 resolutions = simd_make_float3(resolution[0], resolution[1], resolution[2]);
    const float r = resolution[0];
    simd::float3 offset = 0.5 * (dimension * resolutions);
    int n2 = n * n;
//  int num_threads = omp_get_num_threads();
//#pragma omp parallel for shared(index) collapse(3) num_threads(num_threads)
    for (int i = 0; i<n-1; i++) {
        for (int j = 0; j<n-1; j++) {
            for (int k = 0; k<n-1; k++) {
                int i0 = i*n2 + j*n + (k+1);
                int i1 = (i+1)*n2 + j*n + (k+1);
                int i2 = (i+1)*n2 + j*n + k;
                int i3 = i*n2 + j*n + k;
                int i4 = i*n2 + (j+1)*n + (k+1);
                int i5 = (i+1)*n2 + (j+1)*n + (k+1);
                int i6 = (i+1)*n2 + (j+1)*n + k;
                int i7 = i*n2 + (j+1)*n + k;
                /*
                simd::float3 c0 = create_centroid(i0, n, dimensions, n2, offset);
                simd::float3 c1 = create_centroid(i1, n, dimensions, n2, offset);
                simd::float3 c2 = create_centroid(i2, n, dimensions, n2, offset);
                simd::float3 c3 = create_centroid(i3, n, dimensions, n2, offset);
                simd::float3 c4 = create_centroid(i4, n, dimensions, n2, offset);
                simd::float3 c5 = create_centroid(i5, n, dimensions, n2, offset);
                simd::float3 c6 = create_centroid(i6, n, dimensions, n2, offset);
                simd::float3 c7 = create_centroid(i7, n, dimensions, n2, offset);
                */
                simd::float3 c0 = create_centroid(i0, r, dimension) - offset;
                simd::float3 c1 = create_centroid(i1, r, dimension) - offset;
                simd::float3 c2 = create_centroid(i2, r, dimension) - offset;
                simd::float3 c3 = create_centroid(i3, r, dimension) - offset;
                simd::float3 c4 = create_centroid(i4, r, dimension) - offset;
                simd::float3 c5 = create_centroid(i5, r, dimension) - offset;
                simd::float3 c6 = create_centroid(i6, r, dimension) - offset;
                simd::float3 c7 = create_centroid(i7, r, dimension) - offset;
                /*
                simd::float3 points[8] = {
                    ((simd::float3*) centroids)[i0],
                    ((simd::float3*) centroids)[i1],
                    ((simd::float3*) centroids)[i2],
                    ((simd::float3*) centroids)[i3],
                    ((simd::float3*) centroids)[i4],
                    ((simd::float3*) centroids)[i5],
                    ((simd::float3*) centroids)[i6],
                    ((simd::float3*) centroids)[i7]
                };
                 */
                simd::float3 points[8] = {c0, c1, c2, c3, c4, c5, c6, c7};
                float values[8] = {
                    /*
                    fabs( ((Voxel*) voxels)[i0].sdf ),
                    fabs( ((Voxel*) voxels)[i1].sdf ),
                    fabs( ((Voxel*) voxels)[i2].sdf ),
                    fabs( ((Voxel*) voxels)[i3].sdf ),
                    fabs( ((Voxel*) voxels)[i4].sdf ),
                    fabs( ((Voxel*) voxels)[i5].sdf ),
                    fabs( ((Voxel*) voxels)[i6].sdf ),
                    fabs( ((Voxel*) voxels)[i7].sdf )
                    */
                    ( ((Voxel*) voxels)[i0].sdf ),
                    ( ((Voxel*) voxels)[i1].sdf ),
                    ( ((Voxel*) voxels)[i2].sdf ),
                    ( ((Voxel*) voxels)[i3].sdf ),
                    ( ((Voxel*) voxels)[i4].sdf ),
                    ( ((Voxel*) voxels)[i5].sdf ),
                    ( ((Voxel*) voxels)[i6].sdf ),
                    ( ((Voxel*) voxels)[i7].sdf )
                };
                std::vector<simd::float3> temp = polygonise(points, values, isolevel, edgeTable, triTable);
                if (temp.size() == 0) continue;
                /*
                simd::float3* more_triangles = nullptr;
                more_triangles = (simd::float3*) realloc(triangles, (index + temp.size()) * sizeof(simd::float3));
                if ( more_triangles != nullptr )
                    triangles = more_triangles;
                else {
                    free(triangles);
                    return index;
                }
                //triangles = (simd::float3*) realloc(triangles, );
                */
                for (simd::float3 triangle : temp) {
                    ((simd::float3*) triangles)[index][0] = triangle[0];
                    ((simd::float3*) triangles)[index][1] = triangle[1];
                    ((simd::float3*) triangles)[index][2] = triangle[2];
//#pragma omp atomic
                    index++;
                }
            }
        }
    }
    return index;
}

int bridge_integrateDepthMap(float* depthmap,
                             //const void* centroids,
                             const void* extrinsics,
                             const void* intrinsics,
                             void* voxels,
                             const int width,
                             const int height,
                             const int dimension,
                             const float resolution[3],
                             const float delta,
                             const float epsilon,
                             const float lambda) {
    //median_filter(depthmap, 2, width, height);
    // Instanciate all local variables
    int number_of_changes = 0;
    //float diag = 2.0 * sqrt(3.0f) * (resolution[0] / dimension);
    //int count = width * height;
    int size = pow(dimension, 3.0);
    int square = pow(dimension, 2.0);
    simd::float3 resolutions = simd_make_float3(resolution[0], resolution[1], resolution[2]);
    simd::float3 offset = 0.5 * (dimension * resolutions);
    
    // Relative camera variables
    simd_float4x4 K     = ((simd_float4x4 *) intrinsics)[0];
    simd_float4x4 Rt    = ((simd_float4x4 *) extrinsics)[0];
    simd_float4x4 Kinv  = simd_inverse(K);
    simd_float4x4 Rtinv = simd_inverse(Rt);
    simd::float3 ot     = simd_make_float3(Rt.columns[3].x, Rt.columns[3].y, Rt.columns[3].z);
    // Determines bounding box of camera, O(n) where n is width*height of depthmap.
    // It can reduce (always ?) next loop complexity.
    //simd_float2x3 box = compute_bounding_box(depthmap, width, height, Rtinv, Kinv);
    //simd_int3 point_min = global_to_integer(box.columns[0] + offset, resolution, dimensions);
    //simd_int3 point_max = global_to_integer(box.columns[1] + offset, resolution, dimensions);
    //int mini = hash_function(point_min, resolution);
    //int maxi = hash_function(point_max, resolution);
    /*
    for (int i = 0; i<width*height; i++) {
    //for (int i = mini; i<maxi; i++) {
        //if (i >= size || i < 0) continue;
        float depth = depthmap[i];
        int u = i / width;
        int v = i % width;
        simd::float4 homogene = simd_make_float4(u * depth, v * depth, depth, 1);
        simd::float4 global = simd_mul(simd_mul(Rtinv, Kinv), homogene);
        simd::float3 rglobal = simd_make_float3(global.x, global.y, global.z);
        if (! (rglobal.x >= -offset.x && rglobal.y >= -offset.y && rglobal.z >= -offset.z &&
               rglobal.x < offset.x && rglobal.y < offset.x && rglobal.z < offset.z))
            continue;
        simd_int3 coordinate = global_to_integer(rglobal + offset, resolution, dimensions);
        int k = hash_function(coordinate, resolution);
        if (k < 0 || k >= size) continue;
        update_voxel((Voxel *)voxels, 0.0, 1, k);
    }
    */
    
    //for( int i = mini; i<maxi; i++)
    /*
    for (int i = 0; i<size; i++)
    {
        //simd::float3 centroid = create_centroid(i, resolution, dimensions, square, offset);
        simd::float3 centroid = create_centroid(i, resolutions.x, dimension) - offset;
        
        simd::float4 homogene_3d = simd_make_float4(centroid.x, centroid.y, centroid.z, 1);
        float z = simd_distance(ot, centroid);
        //simd::float3 local = simd_mul(rotation, centroid + translation);
        //simd::float4 local = simd_mul(Rt, homogene_3d);
        //simd::float3 local = simd_mul(simd_transpose(rotation), centroid-translation);
        
        //simd::float4 temp = simd_mul(K, local);
        simd::float4 project = simd_mul(simd_mul(K, Rt), homogene_3d);
        int u = (int) (project.x / project.z); //(unhomogene_2d.x / unhomogene_2d.z);
        int v = (int) (project.y / project.z); //(unhomogene_2d.y / unhomogene_2d.z);
        //if (local.z < 0)            continue;
        if (u < 0 || u >= height)   continue;
        if (v < 0 || v >= width)    continue;
        float zp = depthmap[u * width + v];

        // Depth invalid
        if (std::isnan(zp)) continue;
        //if (zp > 1.0) continue;
        
        float distance = zp - z;
        // Calculate weight
        float weight = constant_weighting();
        
        if (fabs(distance) < delta) update_voxel((Voxel *)voxels, distance, weight, i);
        else if (distance >= delta + epsilon)  carving_voxel((Voxel *)voxels, i);
        //else if (distance > delta) update_voxel((Voxel *)voxels, delta, 1, i);
        //else update_voxel((Voxel *)voxels, -delta, 1, i);
    }
     */
    for (int i = 0; i<size; i++)
    {
        //if (i < 0 || i >= size) continue;
        simd::float3 centroid = create_centroid(i, resolutions.x, dimension) - offset;
        simd::float4 homogene_3d = simd_make_float4(centroid.x, centroid.y, centroid.z, 1);
        simd::float4 X_L      = simd_mul(Rt, homogene_3d);
        simd::float4 project = simd_mul(K, X_L);
        
        if (X_L.z < 1e-6) continue;
        
        int u = (int) (project.x / project.z); //(unhomogene_2d.x / unhomogene_2d.z);
        int v = (int) (project.y / project.z); //(unhomogene_2d.y / unhomogene_2d.z);
        //if (local.z < 0)            continue;
        if (u < 0 || u >= height)   continue;
        if (v < 0 || v >= width)    continue;
        
        float z = depthmap[u * width + v];
        // Depth invalid
        if (std::isnan(z)) continue;
        
        //simd::float4 X      = simd_make_float4(u * z, v * z, z, 1);
        //simd::float4 X_S    = simd_mul(Kinv, X);
        //float distance = simd_distance(X_S, X_L);
        //float distance = simd_length(X_S - X_L);
        float distance = X_L.z - z;
        
        //float weight = weighting(distance, delta, epsilon);
        float weight = constant_weighting();
        //distance = fabs(distance) <= delta ? distance : distance > delta ? delta : -delta;
        //update_voxel((Voxel *)voxels, distance, weight, i);
        if (distance >= delta + epsilon)  carving_voxel((Voxel *)voxels, i);
        if (fabs(distance) <= delta) update_voxel((Voxel *)voxels, distance, weight, i);
        //else if (distance >= delta + epsilon)  carving_voxel((Voxel *)voxels, i);
        //else if (fabs(distance) >= delta + epsilon && distance <= X_S.z) carving_voxel((Voxel *)voxels, i);
        //else if (distance >= delta && distance < delta + epsilon) update_voxel((Voxel *)voxels, delta, weight, i);
        //else if (distance < -delta) update_voxel((Voxel *)voxels, -delta, weight, i);
        //else if (distance > delta) update_voxel((Voxel *) voxels, delta, weight, i);
    }
    return number_of_changes;
}

void bridge_exportMeshToPLY(const void* vectors,
                        const char* file_name,
                        int n) {
    save_meshing_ply_format((simd::float3 *) vectors, file_name, n);
}

void bridge_exportVolumeToPLY(const void* centroids,
                        const float* sdfs,
                        const char* file_name,
                        int size) {
    save_volume_ply_format((simd::float3 *) centroids, sdfs, file_name, size);
}

void bridge_fast_icp(const float* previous_points,
                     const float* current_points,
                     const void* intrinsics,
                     void* extrinsics,
                     void* voxels,
                     const int resolution,
                     const float dimension[3],
                     const int width,
                     const int height) {
    simd_float4x4 K = ((simd_float4x4 *) intrinsics)[0];
    simd_float4x4 Rt = ((simd_float4x4 *) extrinsics)[0];
    simd::float3 translation = simd_make_float3(Rt.columns[3].x, Rt.columns[3].y, Rt.columns[3].z);
    simd_float4x4 Kinv = simd_inverse(K);
    simd_float4x4 Rtinv = simd_inverse(Rt);
    //simd::float3 dimensions = simd_make_float3(dimension[0], dimension[1], dimension[2]);
    //int size = pow(resolution, 3.0);
    //int square = pow(resolution, 2.0);
    // Calculate mass center of last point cloud and current point cloud
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
            simd::float4 previous_point  = simd_mul(simd_mul(Rtinv, Kinv), uv);
            previous_count ++;
            previous_mass_centre += simd_make_float3(previous_point.x, previous_point.y, previous_point.z);
        }
        // Current points
        depth = current_points[k];
        if ( ! std::isnan(depth) && depth >= 1e-6 ) {
            simd::float4 uv = simd_make_float4(depth * i, depth * j, depth, 1);
            simd::float4 current_point   = simd_mul(simd_mul(Rtinv, Kinv), uv);
            current_count ++;
            current_mass_centre += simd_make_float3(current_point.x, current_point.y, current_point.z);
        }
    }
    // Divide by number of points seen
    current_mass_centre     = current_mass_centre / (float) current_count;
    previous_mass_centre    = previous_mass_centre / (float) previous_count;
    // Add translation vector between current and previous points to previous transalation
    translation  += (previous_mass_centre - current_mass_centre);
    ((simd_float4x4 *) extrinsics)[0].columns[3] = simd_make_float4(translation.x, translation.y, translation.z, 1);
    /*
    simd::float3 offset = 0.5 * dimensions;
    simd_float3x3 A = simd_diagonal_matrix(simd_make_float3(0,0,0));
    simd::float3 b  = simd_make_float3(0, 0, 0);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j ++) {
            float depth = previous_points[ i*width + j];
            if (std::isnan(depth) || depth < 1e-6) continue;
            simd::float4 uv = simd_make_float4(depth * i, depth * j, depth, 1);
            simd::float4 point  = simd_mul(simd_mul(Rtinv, Kinv), uv);
            simd::float3 rpoint = simd_make_float3(point.x, point.y, point.z);
            simd_int3 V_ijk = global_to_integer(rpoint + offset, resolution, dimensions);
            int k = hash_function(V_ijk, resolution);
            int dkx = k + 1;
            int dky = k + resolution;
            int dkz = k + square;
            if (k >= size || k < 0) continue;
            if (dkx >= size || dkx < 0) continue;
            if (dky >= size || dky < 0) continue;
            if (dkz >= size || dkz < 0) continue;
            simd::float3 gradient = simd_make_float3(((Voxel *) voxels)[dkx].sdf - ((Voxel *) voxels)[k].sdf,
                                                     ((Voxel *) voxels)[dky].sdf - ((Voxel *) voxels)[k].sdf,
                                                     ((Voxel *) voxels)[dkz].sdf - ((Voxel *) voxels)[k].sdf);
            
            for (int n = 0; n < 3; n++)
                for (int m = 0; m < 3; m++)
                    A.columns[n][m] += gradient[n] * gradient[m];
            b += ((Voxel *) voxels)[k].sdf * gradient;
        }
    }
    translation -= simd_mul(simd_inverse(A), b);
    ((simd_float4x4 *) extrinsics)[0].columns[3] = simd_make_float4(translation.x, translation.y, translation.z, 1);
     */
}


void bridge_median_filter(float* depthmap,
                          const int window_size,
                          const int width,
                          const int height) {
    median_filter(depthmap, window_size, width, height);
}

void bridge_drift_correction(const float* current_points,
                             const void* intrinsics,
                             void* extrinsics,
                             const void* voxels,
                             const int dimension,
                             const float resolution[3],
                             const int width,
                             const int height) {
    simd_float3x3 K     = ((simd_float3x3 *) intrinsics)[0];
    simd_float4x3 Rt    = ((simd_float4x3 *) extrinsics)[0];
    simd_float3x3 Kinv  = simd_inverse(K);
    simd_float3x3 rotation  = simd_matrix(Rt.columns[0], Rt.columns[1], Rt.columns[2]);
    simd_float3 translation = Rt.columns[3];
    float offset    = resolution[0] * 0.5;
    int square      = pow(dimension, 2.0);
    int size        = pow(dimension, 3.0);
    simd_float4x4 xtilde = simd_diagonal_matrix(simd_make_float4(0, 0, 0, 0));
    simd_float3x4 xy;
    simd::float3 resolutions = simd_make_float3(resolution[0], resolution[1], resolution[2]);
    for (int i=0; i<height; i++)
    {
        for (int j = 0; j<width; j++)
        {
            float depth = current_points[ i*width + j];
            if ( ! std::isnan(depth) && depth > 1e-6)
            {
                simd::float3 uv = simd_make_float3(i, j, 1);
                simd::float3 global  = simd_mul(simd_transpose(rotation), simd_mul(Kinv, depth * uv) - translation);
                /*simd_int3 global_int = simd_make_int3(
                                                      global_to_integer(global.x + offset, dimension, resolution[0]),
                                                      global_to_integer(global.y + offset, dimension, resolution[1]),
                                                      global_to_integer(global.z + offset, dimension, resolution[2]));
                */
                simd_int3 global_int = global_to_integer(global + offset, resolutions);
                int k = hash_code(global_int, dimension);
                if (k >= size || k < 0) continue;
                float phi_p = ((Voxel *) voxels)[k].sdf;
                simd_float3 M = simd_make_float3(0, 0, 0);
                M[0] = ((Voxel *) voxels)[k + 1].sdf - phi_p;
                M[1] = ((Voxel *) voxels)[k + dimension].sdf - phi_p;
                M[2] = ((Voxel *) voxels)[k + square].sdf - phi_p;
                if (simd_length(M) == 0) continue;
                M = (- M / simd_length(M)) * phi_p;
                for (int n = 0; n<4; n++)
                    for (int m = 0; m<4; m++)
                        xtilde.columns[m][n] += global[n] * global[m];
                for (int n = 0; n<4; n++)
                    for (int m=0; m<3; m++)
                        xy.columns[m][n] += global[n] * M[m];
            }
        }
    }
    //xtilde = simd_inverse(xtilde);
    //simd_float4x3 error = simd_transpose(simd_mul(xtilde, xy));
}

int bridge_raycastDepthMap(float* depthmap,
                           //const void* centroids,
                           const void* extrinsics,
                           const void* intrinsics,
                           void* voxels,
                           const int width,
                           const int height,
                           const int resolution,
                           const float dimension[3],
                           const float delta,
                           const float epsilon,
                           const float lambda) {
    int size = pow(resolution, 3.0);
    int square = pow(resolution, 2.0);
    simd::float3 dimensions = simd_make_float3(dimension[0], dimension[1], dimension[2]);
    simd::float3 offset = 0.5 * (resolution * dimensions);
    simd_float4x4 K = ((simd_float4x4 *) intrinsics)[0];
    simd_float4x4 Kinv = simd_inverse(K);
    simd_float4x4 Rt = ((simd_float4x4 *) extrinsics)[0];
    simd_float4x4 Rtinv = simd_inverse(Rt);
    
    simd::float3 translation = simd_make_float3(Rt.columns[3].x, Rt.columns[3].y, Rt.columns[3].z);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            float depth = depthmap[ i*width + j];
            if (std::isnan(depth) || depth < 1e-6) continue;
            simd::float4 uv = simd_make_float4(depth * i, depth * j, depth, 1);
            simd::float4 global     = simd_mul(simd_mul(Rtinv, Kinv), uv);
            simd::float3 rglobal    = simd_make_float3(global.x, global.y, global.z);
            simd_int3 end           = global_to_integer(rglobal + offset, dimension[0]);
            simd_int3 current       = global_to_integer(translation + offset, dimension[0]);
            //float maxDist = simd_length(direction);
            float dx = global.x - translation.x;
            float dy = global.y - translation.y;
            float dz = global.z - translation.z;
            int stepX = dx == 0 ? 0 : dx > 0 ? 1 : -1;
            int stepY = dy == 0 ? 0 : dy > 0 ? 1 : -1;
            int stepZ = dz == 0 ? 0 : dz > 0 ? 1 : -1;

            if (stepX == 0 && stepY == 0 && stepZ == 0) return 0;
            while (true) {
                if (current.x == end.x && current.y == end.y && current.z == end.z) break;
                int k = hash_code(current, resolution);
                if (k < size && k >= 0) {
                    simd::float3 c = create_centroid(k, dimensions.x, resolution) - offset;
                    float distance = c.z - global.z;
                    float w = constant_weighting();
                    if (distance >= delta + epsilon)  carving_voxel((Voxel *)voxels, i);
                    if (fabs(distance) < delta) update_voxel((Voxel *)voxels, distance, w, k);
                    //if (fabs(distance) <= delta) update_voxel((Voxel *)voxels, distance, w, i);
                }
                if (current.x != end.x)
                    current.x += stepX;
                if (current.y != end.y)
                    current.y += stepY;
                if (current.z != end.z)
                    current.z += stepZ;
            }
        }
    }
    return 0;
}
