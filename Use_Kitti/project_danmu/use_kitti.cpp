#include<iostream>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/Core>
#include <sophus/se3.h>
#include <opencv2/opencv.hpp>

#include <pangolin/pangolin.h>
#include <boost/format.hpp>
#include <fstream>

#include <opticalflow.h>
#include <DrawTrajectory.h>
#include <inpute.h>
#include <poseestimate_2d2d.h>
#include <camare.h>
#include <thread>
#include <triangulation.h>
/*
 time1 : 2020-2-19
 change1 : the initial of the sysyem
 */

using namespace std;
using namespace cv;

extern use_kitti::Camare camare;

int main (int argc, char ** argv)
{

    Mat left_image ;
    Mat right_image ;


    vector<Mat> left_image_squence;
    vector<Mat> right_image_squence;

    vector_M vector_M_GT;                  //ground truth  matrix M
    vector_P vector_P_GT, vector_P_e;                  //ground truth  translation vector
//    vector_P_mat vector_P_e;               //estimated translation vector
    vector_P points;
    vector<vector_P> points_v;

    bool success= true;
    success=inpute(left_image_squence,right_image_squence,vector_M_GT,vector_P_GT,1000); // inpute the data
    if(success== true)
    {
        cout<<"inpute success"<<endl<<"the number of inputed image is :"<<left_image_squence.size()<<endl;
    } else{
        cout<<"inpute not success"<<endl;
        return 1;
    }

//    显示读取到的图像
//    waitKey(0);
//    for(int i=0;i<left_image_squence.size();i++)
//    {
//        imshow("left",left_image_squence[i]);
//        imshow("right",right_image_squence[i]);
//        waitKey(1);
//    }

//    left_image=imread("./data/00/image_0/000000.png",0);
//    right_image=imread("./data/00/image_1/000000.png",0);
//    Mat left_image1=imread("./data/00/image_0/000002.png",0);

//    imshow("left",left_image);
//    imshow("right",right_image);
//    imshow("left1",left_image1);

    Mat R,deta_t;
    Mat t=(Mat_<double>(3,1)<<0,0,0);//  give a inital value to t ,ortherwise  thr function DrawTrajectory will fail
    Eigen::Vector3d t2,t_sum,b;
    Eigen::Matrix3d R2,R_sum;
    b<<0,0,0;
    vector_P_e.push_back(b);   //   give a inital value to vector_P_e ,ortherwise  thr function DrawTrajectory will fail
    points.push_back(b);       //   give a inital value to points ,ortherwise  thr function DrawTrajectory will fail
    points_v.push_back(points);  //  point is a vector wich contain 3D coordinate value from one image  ,and  points_v is a vector contain points  which include all the keypoints' 3D coordinate value

    // a thread which to draw the DrawTrajectory and keypoiuts
    thread task01(DrawTrajectory,std::ref(vector_P_GT),std::ref(vector_P_e));

    for(int i=0;i<left_image_squence.size()-1;i++)//left_image_squence.size()
    {
        cout<<"***************loop"<<i<<"***************"<<endl;
        poseestimate_2d2d(left_image_squence[i+1],left_image_squence[i],R,deta_t);
        if(i==0){
            R2<<R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),
            R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),
            R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2);

            t2<<deta_t.at<double>(0,0),deta_t.at<double>(1,0),deta_t.at<double>(2,0);

            R_sum=R2;//  initial value of R_sum \R2\ t_sum and t2
            t_sum=t2;

            t2=R2*t2;
           //t3=R*t2;
            vector_P_e.push_back(t2);
            //t=t+deta_t;
//            cout<<"estimated translation vector t1 is "<<t<<endl;
            continue;
        }
        R2<<R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),
                R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),
                R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2);

        t2<<deta_t.at<double>(0,0),deta_t.at<double>(1,0),deta_t.at<double>(2,0);

        R_sum=R2*R_sum;   // rotation matrix of ith fram campared to the first fram( world coordinate system)

        cout<<"R_sum:"<<R_sum<<endl;
        cout<<"R:"<<R<<endl;
        t_sum=t_sum+R_sum*t2;

//        t2<<t.at<double>(0,0),t.at<double>(1,0),t.at<double>(2,0);
        cout <<"estimated translation vector t_sum is "<<endl<<t_sum<<endl;
 //       cout <<"estimated translation vector deta_t is "<<endl<<deta_t<<endl;
        vector_P_e.push_back(t_sum);


        //   三角化恢复3d点
//        triangulation_main(left_image_squence[i],left_image_squence[i+1],points);
//        for(int j=0;j<points.size();j++)
//        {
//            points[j]=R_sum*points[j];
//        }
//        points_v.push_back(points);

       // usleep(100000);
    }
    task01.join();

    cout<<"number of estimated poses"<<vector_P_e.size();
    //save the trajectory to trajectory.txt
    ofstream outfile;
    outfile.open("./trajectory.txt",ios::out);
    if(!outfile)
    {
        cout<<"can not open trajectory.txt"<<endl;
    }
    for(int i=0;i<vector_P_e.size();i++)
    {
        outfile<<i<<":"<<endl;
//        cout<<vector_P_e[i]<<endl;
        outfile<<vector_P_e[i]<<endl;
    }
    outfile.close();
//    DrawTrajectory(vector_P_GT,vector_P_e);





//    Mat depth_image=Mat::zeros(left_image.size(),left_image.type());
//
////    imshow("depth", depth_image);
////    waitKey(0);
//    for(int v=1;v<left_image.rows;v++)//v  行数  rows 与y平行
//    {
//        unsigned char* row_ptr=depth_image.ptr<unsigned char> (v);//把图像的第v行指针赋给row_ptr
//        for(int u=1;u<left_image.cols;u++ )//列数   cols  与x平行
//        {
//            unsigned char* data_ptr=&row_ptr[u*depth_image.channels()];
//            *data_ptr=left_image.at<uchar >(v,u);
//            cout<<left_image.at<double>(v,u)<<endl;
//        }
//    }
//
//
//    cout<<"v  行数"<<left_image.rows<<endl<<"u  列数"<<left_image.cols<<endl;
////    imshow("depth",depth_image);
////    waitKey(0);
//
//    vector<KeyPoint> kp1;
//    Ptr<GFTTDetector> detector = GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
//    detector->detect(left_image, kp1);
//
//
//    vector<KeyPoint> kp2_single;
//    vector<bool> success_single;
//
//    OpticalFlowSingleLevel(left_image, right_image, kp1, kp2_single, success_single);
//
//    Mat img2_single;
//    cv::cvtColor(right_image, img2_single, CV_GRAY2BGR);
//    for (int i = 0; i < kp2_single.size(); i++) {
//        if (success_single[i]) {
//            cv::circle(img2_single, kp2_single[i].pt, 2, cv::Scalar(0, 0, 250), 2);
//            cv::line(img2_single, kp1[i].pt, kp2_single[i].pt, cv::Scalar(0, 250, 0));
//        }
//    }
//
//    Mat img1_CV;
//    cv::cvtColor(left_image,img1_CV,CV_GRAY2BGR);
//    for (int i = 0; i < kp1.size(); i++) {
//        if (success_single[i]) {
//            cv::circle(img1_CV, kp1[i].pt, 2, cv::Scalar(0, 250, 0), 2);
//            cv::line(img1_CV, kp1[i].pt, kp2_single[i].pt, cv::Scalar(0, 250, 0));
//        }
//    }
//
//    imshow("left",img2_single);
//    imshow("right", img1_CV);
//    waitKey(0);


    return 0;
}









