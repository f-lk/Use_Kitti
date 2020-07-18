//
// Created by flk on 20-2-13.
//

#ifndef USE_KITTI_POSEESTIMATE_2D2D_H
#define USE_KITTI_POSEESTIMATE_2D2D_H

#include <inpute.h>
using namespace std;
using namespace cv;

//// estimate the R,t from img_1 to img_2
void poseestimate_2d2d ( Mat & img_1,Mat & img_2, Mat & R, Mat & t );

//// estimate the R,t from keypoints_1 from img_1 to keypoints_2 from img_2
void pose_estimation_2d2d ( std::vector<KeyPoint> keypoints_1,//******vector的作用相当于建立了一个KeyPoint的动态数组
                            std::vector<KeyPoint> keypoints_2,//*****定义了三个动态数组
                            std::vector< DMatch > matches,// the matched points
                            Mat& R, Mat& t );

//// find matched points fo img_1 and img_2
void find_feature_matches(const Mat& img_1, const Mat& img_2,  //inpute image
                            std::vector<KeyPoint>& keypoints_1, //keypoints_1 from image1
                            std::vector<KeyPoint>& keypoints_2, //keypoints_2 from image2
                            std::vector< DMatch >& matches );

#endif //USE_KITTI_POSEESTIMATE_2D2D_H
