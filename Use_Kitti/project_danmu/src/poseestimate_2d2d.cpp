//
// Created by flk on 20-2-13.
//

#include <poseestimate_2d2d.h>
#include <camare.h>

use_kitti::Camare camare;


void find_feature_matches (
        const Mat& img_1, const Mat& img_2,
        std::vector<KeyPoint>& keypoints_1,
        std::vector<KeyPoint>& keypoints_2,
        std::vector< DMatch >& matches );

void pose_estimation_2d2d (
        std::vector<KeyPoint> keypoints_1,
        std::vector<KeyPoint> keypoints_2,
        std::vector< DMatch > matches,
        Mat& R, Mat& t );

// 像素坐标转相机归一化坐标

void poseestimate_2d2d ( Mat & img_1,Mat & img_2, Mat & R, Mat & t )
{
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );//ORB特征点
//    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;

    //-- 估计两张图像间运动
    pose_estimation_2d2d ( keypoints_1, keypoints_2, matches, R, t );

    //-- 验证E=t^R*scale
    Mat t_x = ( Mat_<double> ( 3,3 ) <<
            0,                      -t.at<double> ( 2,0 ),     t.at<double> ( 1,0 ),//××t.at<1,0>是t中第二行第一列的元素
            t.at<double> ( 2,0 ),      0,                      -t.at<double> ( 0,0 ),
            -t.at<double> ( 1,0 ),     t.at<double> ( 0,0 ),      0 );

//    cout<<"t^R="<<endl<<t_x*R<<endl;//*******t^R和E相差一个因子

    //-- 验证对极约束
//    Mat K = ( Mat_<double> ( 3,3 ) << 718.86, 0, 607.19, 0, 718.86, 185.22, 0, 0, 1 );
//    camare.K;
//    cout<<"K="<<endl<<camare.K<<endl;
//    cout<<camare.K(0,0)<<endl;
    //   printf("%d",K.at<double>(0,0));
    for ( DMatch m: matches )//××对匹配上的特征点进行遍历
    {
        Point2d pt1 = camare.pixel2cam ( keypoints_1[ m.queryIdx ].pt);//××将p1转化为x1//queryIdx是图片img1中进行匹配的序号
        Mat y1 = ( Mat_<double> ( 3,1 ) << pt1.x, pt1.y, 1 );//××为啥是x,y//queryIdx : 查询点的索引（当前要寻找匹配结果的点在它所在图片上的索引）.trainIdx : 被查询到的点的索引（存储库中的点的在存储库上的索引）于是 queryIdx 是你想要为“它”找到匹配结果的“它”的索引。 trainIdx是 “它”的匹配结果的索引。//百度搜素Dmatch
//        cout<<pt1.x<<endl;
        Point2d pt2 = camare.pixel2cam ( keypoints_2[ m.trainIdx ].pt);//××trainIdx是图片img2中与图片img1中queryIdx对应的序号
        Mat y2 = ( Mat_<double> ( 3,1 ) << pt2.x, pt2.y, 1 );
        Mat d = y2.t() * t_x * R * y1;
//        cout << "epipolar constraint = " << d << endl;
    }

}


void find_feature_matches ( const Mat& img_1,
                            const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );
    Mat outimg_1,outimg_2;
    drawKeypoints( img_1, keypoints_1, outimg_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    drawKeypoints( img_2, keypoints_2, outimg_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
//    imshow("key points",outimg_1);
//    imshow("key points_2",outimg_2);
//    waitKey(0);
    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
//
//    printf ( "-- Max dist : %f \n", max_dist );
//    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
    //     画出运行时的图像
    Mat img_match;
    drawMatches(img_1,keypoints_1,img_2,keypoints_2,matches,img_match);
    Mat dst_1;
    resize(img_match,dst_1,Size(),0.7,0.7);
//    imshow("matching keypoint",dst_1);
//    waitKey(2);
}




void pose_estimation_2d2d ( std::vector<KeyPoint> keypoints_1,//******vector的作用相当于建立了一个KeyPoint的动态数组
                            std::vector<KeyPoint> keypoints_2,//*****定义了三个动态数组
                            std::vector< DMatch > matches,
                            Mat& R, Mat& t )
{
    // 相机内参,TUM Freiburg2
    Mat K = ( Mat_<double> ( 3,3 ) << 718.86, 0, 607.19, 0, 718.86, 185.22, 0, 0, 1 );

    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;//***Point2f是一个类，看的不是太懂,用来存放提取到的关键点
    vector<Point2f> points2;

    // 将match好的点取出来   使得points1[i]和points2[i]为对应的匹配点
    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );//于是 queryIdx 是你想要为“它”找到匹配结果的“它”的索引。 trainIdx是 “它”的匹配结果的索引。
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }
//    cout<<matches.size()<<endl;
    //-- 计算基础矩阵F
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat ( points1, points2, CV_FM_8POINT );//八点法，这个函数的后面两个参数可要可不要
//    cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    //-- 计算本质矩阵E
    Point2d principal_point ( 607.19, 185.22 );	//相机光心
    double focal_length = 718.86;			//相机焦距
    Mat essential_matrix;
    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point, RANSAC );
//    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    //-- 计算单应矩阵
    Mat homography_matrix;
    homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
//    cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );//the transformate  for points2  to point1
//    cout<<"R is "<<endl<<R<<endl;
//    cout<<"t is "<<endl<<t<<endl;

}

