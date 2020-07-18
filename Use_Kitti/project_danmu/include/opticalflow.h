//
// Created by flk on 20-2-12.
//

#ifndef USE_KITTI_OPTICALFLOW_H
#define USE_KITTI_OPTICALFLOW_H


#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>

using namespace std;
using namespace cv;

////  OpticalFlow
void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse= false
);

#endif //USE_KITTI_OPTICALFLOW_H
