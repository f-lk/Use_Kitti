//
// Created by flk on 20-2-13.
//
#include <camare.h>

namespace use_kitti
{
    Point2d Camare::pixel2cam ( const Point2d& p)
    {
        return Point2d
                (
                        ( p.x - cx ) / fx,
                        ( p.y - cy ) / fy
                );
    }
}


