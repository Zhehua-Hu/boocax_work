#include "solve_pnp.h"
void solve_pnp(vector<Point3f> objectPoints, vector<Point2f> imagePoints,
               InputArray cameraMatrix, InputArray distCoeffs,
               bool useExtrinsicGuess, int flags,int radius){
    if(imagePoints.size()!=4)  return;
    vector<Mat> set_R ,set_t;
    vector<Mat> set_center;
    vector<Point2f> p1,p2,p3,p4,point2d;
    Mat R,center;
    for(int u = -radius;u<=radius;u++){
        for(int v = -radius;v<=radius;v++){
            p1.push_back(Point2f(imagePoints[0].x+u,imagePoints[0].y+v));
            p2.push_back(Point2f(imagePoints[1].x+u,imagePoints[1].y+v));
            p3.push_back(Point2f(imagePoints[2].x+u,imagePoints[2].y+v));
            p4.push_back(Point2f(imagePoints[3].x+u,imagePoints[3].y+v));
        }
    }

    for (auto p1_1 : p1){
        for(auto p2_2 : p2){
            for(auto p3_3 : p3){
                for(auto p4_4 :p4){
                    point2d.push_back(p1_1);
                    point2d.push_back(p2_2);
                    point2d.push_back(p3_3);
                    point2d.push_back(p4_4);
                    Mat r,t;
                    solvePnP(objectPoints,point2d,cameraMatrix,distCoeffs, r,t,useExtrinsicGuess,flags);
                    Rodrigues(r,R);
                    center = -R.t()*t;
                    set_R.push_back(R);
                    set_t.push_back(t);
                    set_center.push_back(center);
                    point2d.clear();
                }
            }
        }
    }

    vector<vector<double>> pose,drawed_pose;
    for (int i=0;i<set_R.size();i++){
        //定义一个暂时的pose_temp存储12个位姿数据，9个旋转矩阵的元素，3各位置元素  R_temp.type()<<endl<<
        cout<<"=========================================="<<endl;
            vector<double> pose_temp;
            Mat R_temp = set_R[i],t_temp = set_t[i];
//          cout<<t_temp<<endl;
//            cout<<R_temp.at<double>(0,0)<<endl<<R_temp<<endl;
            pose_temp.push_back(R_temp.at<double>(0,0));	pose_temp.push_back(R_temp.at<double>(1,0));	pose_temp.push_back(R_temp.at<double>(2,0));
            pose_temp.push_back(R_temp.at<double>(0,1));	pose_temp.push_back(R_temp.at<double>(1,1));	pose_temp.push_back(R_temp.at<double>(2,1));
            pose_temp.push_back(R_temp.at<double>(0,2));	pose_temp.push_back(R_temp.at<double>(1,2));	pose_temp.push_back(R_temp.at<double>(2,2));
            //位置元素
            pose_temp.push_back(t_temp.at<double>(0,0)/10.0);	 pose_temp.push_back(t_temp.at<double>(1,0)/10.0);pose_temp.push_back(t_temp.at<double>(2,0)/10.0);
            //将pose_temp存入全局变量pose用于构图，也就是每一行的pose都是一个pose_temp，12个数，最后会有index行
            pose.push_back(pose_temp);
            //清空pose_temp内存
            pose_temp.clear();
    }

    for (int i=0;i<set_R.size();i++){
        if(i%100 ==0)   drawed_pose.push_back(pose[i]);
    }

//    draw_camera(objectPoints,drawed_pose);

    cv::waitKey(0);
//    Mat center_average =set_center[0];
    //求解中心位置的均值

//    for(int i=1;i<set_center.size();i++){
//        center_average = center_average/(i+1)*i+set_center[i]/(i+1);
//        cv::add(set_center[i],center_average,center_average);
//        cout<<"center_average:"<<endl<<center_average<<endl;

//    }

//    center_average = center_average/set_center.size();
//    cout<<"center_average:"<<endl<<center_average<<endl;
//    cout<<"============================"<<endl;
//    solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeffs, r,t,useExtrinsicGuess,flags);
//    Rodrigues(r,R);
//    center = -R.t()*t;
//    cout<<"center:"<<endl<<center<<endl;
}

//void draw_camera(vector<Point3f> objectPoints,vector<vector<double>> pose_fin)
//{
//       const float w=0.2;
//       const float h=w*0.75;
//       const float z=w*0.6;
//       //生成一个gui界面，定义大小
//       pangolin::CreateWindowAndBind("Main",1240,860);
//       //进行深度测试，保证某视角下像素只有一种颜色，不混杂
//       glEnable(GL_DEPTH_TEST);

//       //放置一个相机
//       pangolin::OpenGlRenderState s_cam(
//           pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
//           pangolin::ModelViewLookAt(20,1.5,0, 0,1.5,0, pangolin::AxisY)
//       );

//       //创建视角窗口
//       pangolin::Handler3D handler(s_cam);
//       pangolin::View& d_cam = pangolin::CreateDisplay()
//               .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
//               .SetHandler(&handler);
//       glClearColor(1.0f,1.0f,1.0f,1.0f);

//       while( !pangolin::ShouldQuit() )
//       {
//           //清除颜色缓冲和深度缓冲
//           glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//           d_cam.Activate(s_cam);

//       //背景先弄成白色的吧，我觉得白色比较好看
//           glColor3f(1.0f,0.50f,1.0f);
//           glLineWidth(4);
//           glBegin(GL_LINES);
//           glColor3f(0.0f,0.0f,0.0f);
//           glVertex3f(0,0,0);		glVertex3f(0,3,0);
//           glVertex3f(0,0,0);		glVertex3f(10,0,0);
//           glVertex3f(0,3,0);		glVertex3f(10,3,0);
//           glVertex3f(10,0,0);		glVertex3f(10,3,0);
//           glEnd();

//       for(int i=0;i<pose_fin.size();i++)
//       {
//           //使用位置变换矩阵
//           glPushMatrix();
//           //变换如该矩阵，注意这个变换矩阵是转置的
//           std::vector<GLdouble> Twc ={pose_fin[i][0],pose_fin[i][1],pose_fin[i][2],0,
//                                     pose_fin[i][3],pose_fin[i][4],pose_fin[i][5],0,
//                                     pose_fin[i][6],pose_fin[i][7],pose_fin[i][8],0,
//                                     pose_fin[i][9],pose_fin[i][10],pose_fin[i][11],1 };

//           //变换
//           glMultMatrixd(Twc.data());
//           //每次变换后绘制相机
//           glLineWidth(2);
//           glBegin(GL_LINES);
//           glColor3f(0.0f,0.0f,0.0f);
//           glVertex3f(0,0,0);		glVertex3f(w,h,z);
//           glVertex3f(0,0,0);		glVertex3f(w,-h,z);
//           glVertex3f(0,0,0);		glVertex3f(-w,-h,z);
//           glVertex3f(0,0,0);		glVertex3f(-w,h,z);
//           glVertex3f(w,h,z);		glVertex3f(w,-h,z);
//           glVertex3f(-w,h,z);		glVertex3f(-w,-h,z);
//           glVertex3f(-w,h,z);		glVertex3f(w,h,z);
//           glVertex3f(-w,-h,z);		glVertex3f(w,-h,z);

//           glEnd();
//           glPopMatrix();
//       }

//       //绘制连接的绿色线
////       glLineWidth(2);
////           glBegin ( GL_LINES );
////       glColor3f ( 0.0f,1.f,0.f );
////       for(int i=0;i<pose_fin.size()-1;i++)
////       {
////           glVertex3f( pose_fin[i][9],pose_fin[i][10],pose_fin[i][11]);
////           glVertex3f( pose_fin[i+1][9],pose_fin[i+1][10],pose_fin[i+1][11] );
////       }
////       glEnd();

//           //交换帧和并推进事件
//        pangolin::FinishFrame();
//       }

//}
