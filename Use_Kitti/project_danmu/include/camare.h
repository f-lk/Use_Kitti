//
// Created by flk on 20-2-13.
//

#ifndef USE_KITTI_CAMARE_H
#define USE_KITTI_CAMARE_H

#include <poseestimate_2d2d.h>

//a camare include its intrinsic parameters

namespace use_kitti
{
class Camare
{
protected:
    double fx=718.86,fy=718.86,cx=607.19,cy=185.22;


public:
    Eigen::Matrix3d K;
    Camare()
    {
        K<< fx,0,cx,
                0,fy,cy,
                0, 0,1;
    }
    Point2d pixel2cam ( const Point2d& p );
};



}


#endif //USE_KITTI_CAMARE_H
