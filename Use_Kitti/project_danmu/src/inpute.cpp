//
// Created by flk on 20-2-12.
//
#include <inpute.h>

using namespace cv;

bool inpute(vector<Mat> & left_image_squence,       //左侧图像序列
            vector<Mat> & right_image_squence,      //右侧图像序列
            vector_M & vector_M_GT,                  //ground truth  matrix M
            vector_P & vector_P_GT,                   //ground truth  translation vector
            int number
            )
{
    Mat left_image,right_image;

    boost::format fmt ("/home/flk/桌面/dataset/kiiti/odometry/data_odometry_gray/00/image_0/%06d.png");
    boost::format fmt2 ("/home/flk/桌面/dataset/kiiti/odometry/data_odometry_gray/00/image_1/%06d.png");

    ifstream fin("/home/flk/桌面/dataset/kiiti/odometry/poses/00.txt");

    if (!fin)
    {
        cerr<<"not found poses_00.txt"<<endl;
        return false;
    }
    cout <<"image inputing"<<endl;
    for(int i=0;i<number;i++)
    {
        left_image= imread((fmt%i).str(),0);
        right_image= imread((fmt2%i).str(),0);
        //cout<<"path"<<(fmt%i).str();
        if(left_image.empty()||right_image.empty()){ continue;}

        left_image_squence.push_back(left_image);
        right_image_squence.push_back(right_image);

//        imshow("left image",left_image);
//        imshow("right image",right_image);

//        waitKey(2);
    }

    for (int i=0;i<number;i++)
    {
        double data[12]={0};
        for(auto& d:data)
            fin>>d;
        Eigen::Matrix<double ,3,4> Matrix ;
        Matrix<<data[0],data[1],data[2],data[3],
                data[4],data[5],data[6],data[7],
                data[8],data[9],data[10],data[11];
        vector_M_GT.push_back(Matrix);
        Eigen::Vector3d vector_GN;
        vector_GN<<data[3],data[7],data[11];
        vector_P_GT.push_back(vector_GN);
    }

    int r=rand()%(number-100)+100;
    if(left_image_squence[r].empty()||right_image_squence[r].empty())
    {
        cout<<"not found left or right"<<endl;
        return false;
    }

    cout<<"number of pose "<< vector_P_GT.size()<<endl;
    cout<<"number of images "<<left_image_squence.size()<<endl;
    return true;
}


