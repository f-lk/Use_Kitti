//
// Created by flk on 20-2-17.
//
#include <triangulation.h>
using namespace use_kitti;

extern Camare camare;

void triangulation (
        const vector< KeyPoint >& keypoint_1,
        const vector< KeyPoint >& keypoint_2,
        const std::vector< DMatch >& matches,
        const Mat& R, const Mat& t,
        vector_P& points );

void triangulation_main(  Mat& img_1, Mat& img_2, vector_P& points)
{
    //-- 读取图像
//    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
//    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );
//    imshow("img_1",img_1);
//    imshow("img_1",img_2);

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );//***keypoints_1, keypoints_2,存放关键点的二维坐标信息，matches 存放关键点之间的匹配关系。
    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;
//    cout<<keypoints_1[1].pt<<endl;
//    waitKey(0);
    //-- 估计两张图像间运动
    Mat R,t;
    pose_estimation_2d2d ( keypoints_1, keypoints_2, matches, R, t );//根据两张图片关键点的信息得出图片之间的运动过程，R，t

    //-- 三角化
 //   vector<Point3d> points;
    triangulation( keypoints_1, keypoints_2, matches, R, t, points );//根据两张图片的关键点的信息，和R，t，计算得出  关键点的三维信息，放到points中  points 中点的坐标是以图像一的相机为坐标系的

    //-- 验证三角化点与特征点的重投影关系
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    for ( int i=0; i<matches.size(); i++ )
    {
        Point2d pt1_cam = camare.pixel2cam(keypoints_1[ matches[i].queryIdx ].pt);//××像素坐标转化为相机坐标//×××keypoints_1[ matches[i].queryIdx ].pt  是   已经匹配好的第matches[i].queryIdx点的坐标
        Point2d pt1_cam_3d(
                points[i][0]/points[i][2], //×××在相机坐标系的归一化平面上
                points[i][1]/points[i][2]
        );

        cout<<"point in the first camera frame: "<<pt1_cam<<endl;
        cout<<"point projected from 3D "<<pt1_cam_3d<<", d="<<points[i][2]<<endl;

        // 第二个图
        Point2f pt2_cam = camare.pixel2cam(  keypoints_2[ matches[i].trainIdx ].pt   ) ;
        Mat pt2_trans = R*( Mat_<double>(3,1) << points[i][0], points[i][1], points[i][2] ) + t;
        pt2_trans /= pt2_trans.at<double>(2,0);
        cout<<"point in the second camera frame: "<<pt2_cam<<endl;
        cout<<"point reprojected from second frame: "<<pt2_trans.t()<<endl;
        cout<<endl;
    }

}

void triangulation (
        const vector< KeyPoint >& keypoint_1,
        const vector< KeyPoint >& keypoint_2,
        const std::vector< DMatch >& matches,
        const Mat& R, const Mat& t,
        vector_P & points )
{
    Mat T1 = (Mat_<float> (3,4) <<
                                1,0,0,0,
            0,1,0,0,
            0,0,1,0);
    Mat T2 = (Mat_<float> (3,4) <<
                                R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),//从第一张图片变换到第二章图片所用的矩阵T
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
    );

//    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );//***相机内参
    vector<Point2f> pts_1, pts_2;//用来存放float型的二维坐标
    for ( DMatch m:matches )//****遍历所有匹配点，将像素坐标系下的点坐标转换至相机坐标系下的点坐标，并存放到pts_1, pts_2中
    {
        // 将像素坐标转换至相机坐标
        pts_1.push_back ( camare.pixel2cam( keypoint_1[m.queryIdx].pt) );
        pts_2.push_back ( camare.pixel2cam( keypoint_2[m.trainIdx].pt) );
    }

    Mat pts_4d;
    cv::triangulatePoints( T1, T2, pts_1, pts_2, pts_4d );
//    cout<<pts_4d.cols<<endl;//***输出矩阵的列数
    //   cout<<pts_4d.rows<<endl;//×××输出矩阵的行数
    // 转换成非齐次坐标    //p中存放的是  关键点的三维坐标（齐次坐标形式，转化为非齐次形式，前三维就是唯一的关键点的三维坐标）
    for ( int i=0; i<pts_4d.cols; i++ )
    {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0); // 归一化
        Eigen::Vector3d p (
                x.at<float>(0,0),
                x.at<float>(1,0),
                x.at<float>(2,0)
        );
        points.push_back( p );
    }
}
