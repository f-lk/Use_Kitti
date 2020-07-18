//
// Created by flk on 20-2-12.
//

#include <DrawTrajectory.h>

void DrawTrajectory(vector_P & poses)//vector_P & poses_2
{
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("uuu", 1024, 768);//*****先建立一个空白窗口
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, 100, 0, 0, 0, 0, 0.0, -1.0, 0.0)//第一组eyex, eyey,eyez 相机在世界坐标的位置;第二组centerx,centery,centerz 相机镜头对准的物体在世界坐标的位置;第三组upx,upy,upz 相机向上的方向在世界坐标中的方向
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    cout<<"pose.size"<<poses.size()<<endl;

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() -1; i++) {
            glColor3f(0,0,255);//1 - (float) i / poses.size(), 0.0f, (float) i / poses.size()
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            //cout<<"pose[i]"<<p1<<endl;

            glVertex3d(p1[0], p1[1], p1[2]);//用平移向量进行绘图
            glVertex3d(p2[0], p2[1], p2[2]);
            glEnd();
        }

        glLineWidth(5);
        glColor3f(255,0,0);
        glBegin(GL_LINES);

        glVertex3d(5,0,0);
        glVertex3d(0,0,0);

//            glVertex3d(0,5,0);
//            glVertex3d(0,0,0);
        glEnd();

//            for (size_t i = 0; i < poses_2.size() -1; i++) {
//                glColor3f(0,255,255);//1 - (float) i / poses.size(), 0.0f, (float) i / poses.size()
//                glBegin(GL_LINES);
//                auto p1 = poses_2[i], p2 = poses_2[i + 1];
//                // cout<<"pose[i]"<<p1<<endl;
//
//                glVertex3d(p1[0], p1[1], p1[2]);//用平移向量进行绘图
//                glVertex3d(p2[0], p2[1], p2[2]);
//                glEnd();
//            }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}


void DrawTrajectory(vector_P & poses,vector_P & poses_2)//, vector<vector_P>& points_v
{
    if (poses.empty()||poses_2.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("uuu", 1024, 768);//*****先建立一个空白窗口
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, 400, 0, 0, 0, 0, 0.0, 0, 1)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    cout<<"pose.size"<<poses.size()<<endl;

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);////清除颜色缓冲和深度缓冲

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//	//背景颜色

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() -1; i++) {
            glColor3f(0,0,255);//1 - (float) i / poses.size(), 0.0f, (float) i / poses.size()
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            //cout<<"pose[i]"<<p1<<endl;

            glVertex3d(p1[0], p1[1], p1[2]);//用平移向量进行绘图
            glVertex3d(p2[0], p2[1], p2[2]);
            glEnd();
        }

        glLineWidth(5);
        glColor3f(255,0,0);
        glBegin(GL_LINES);

        glVertex3d(5,0,0);
        glVertex3d(0,0,0);

//            glVertex3d(0,5,0);
//            glVertex3d(0,0,0);
        glEnd();

        for (size_t i = 0; i < poses_2.size() -1; i++) {
            glColor3f(0,255,255);//1 - (float) i / poses.size(), 0.0f, (float) i / poses.size()
            glBegin(GL_LINES);
            auto p1 = poses_2[i], p2 = poses_2[i + 1];
            // cout<<"pose[i]"<<p1<<endl;

            glVertex3d(p1[0], p1[1], p1[2]);//用平移向量进行绘图
            glVertex3d(p2[0], p2[1], p2[2]);
            glEnd();
        }
        glPointSize(2);
        glBegin(GL_POINTS);
        glColor3f(255,0,0);


        //用来画3d点
//        for(int i=0;i<points_v.size();i++)
//        {
//            for(int j=0;j<points_v[i].size();j++)
//            {
//                glVertex3f(points_v[i][j][0],points_v[i][j][1],points_v[i][j][2]);
//            }
//        }



        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}

void DrawTrajectory(vector_P_mat & poses)//vector_P & poses_2
{
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("uuu", 1024, 768);//*****先建立一个空白窗口
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    cout<<"pose.size"<<poses.size()<<endl;

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() -1; i++) {
            glColor3f(0,0,255);//1 - (float) i / poses.size(), 0.0f, (float) i / poses.size()
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            //cout<<"pose[i]"<<p1<<endl;

            glVertex3d(p1.at<double>(0,0), p1.at<double>(1,0), p1.at<double>(2,0));//用平移向量进行绘图
            glVertex3d(p2.at<double>(0,0), p2.at<double>(1,0), p2.at<double>(2,0));
            glEnd();
        }

        glLineWidth(5);
        glColor3f(255,0,0);
        glBegin(GL_LINES);

        glVertex3d(5,0,0);
        glVertex3d(0,0,0);

//            glVertex3d(0,5,0);
//            glVertex3d(0,0,0);
        glEnd();

//            for (size_t i = 0; i < poses_2.size() -1; i++) {
//                glColor3f(0,255,255);//1 - (float) i / poses.size(), 0.0f, (float) i / poses.size()
//                glBegin(GL_LINES);
//                auto p1 = poses_2[i], p2 = poses_2[i + 1];
//                // cout<<"pose[i]"<<p1<<endl;
//
//                glVertex3d(p1[0], p1[1], p1[2]);//用平移向量进行绘图
//                glVertex3d(p2[0], p2[1], p2[2]);
//                glEnd();
//            }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}