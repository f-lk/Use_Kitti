//
// Created by flk on 20-2-12.
//

#ifndef USE_KITTI_INPUTE_H
#define USE_KITTI_INPUTE_H



#include <DrawTrajectory.h>
#include <fstream>
#include <boost/format.hpp>

typedef vector<Eigen::Matrix<double ,3,4>,Eigen::aligned_allocator<Eigen::Matrix<double ,3,4>>>  vector_M;

////  inpute the image and the ground truth
bool inpute(vector<Mat> & left_image_squence,       //左侧图像序列
            vector<Mat> & right_image_squence,      //右侧图像序列
            vector_M & vector_M_GT,                  //ground truth  matrix M
            vector_P & vector_P_GT,                  //ground truth  translation vector
            int number                              //the number of inputed image
            );

#endif //USE_KITTI_INPUTE_H
