//
// Created by flk on 20-2-12.
//

#ifndef USE_KITTI_DRAWTRAJECTORY_H
#define USE_KITTI_DRAWTRAJECTORY_H

#include <opticalflow.h>
#include <pangolin/pangolin.h>
typedef vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> vector_P;
typedef vector<Mat,Eigen::aligned_allocator<Mat>> vector_P_mat;
//// draw the poses of the camare
//void DrawTrajectory(vector_P & poses);//vector_P & poses_2

//// draw the poses of the camare and of the groud trurh
//void DrawTrajectory(vector_P_mat & poses);//vector_P & poses_2

//// draw the poses of the camare and of the groud trurh  ,and the point
void DrawTrajectory(vector_P & poses,vector_P & poses_2);//,vector<vector_P>& points

#endif //USE_KITTI_DRAWTRAJECTORY_H
