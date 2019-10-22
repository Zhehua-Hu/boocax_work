#include "viewer.h"

void Viewer::setVehicle(Vehicle *v){
    vehicle =v;
}

void Viewer::setlandmarks(vector<Vec3f> land_marks){
    landmarks.assign(land_marks.begin(),land_marks.end());
}


void Viewer::Run(){
    //生成一个gui界面，定义大小
    pangolin::CreateWindowAndBind("Main",1240,860);
    //进行深度测试，保证某视角下像素只有一种颜色，不混杂
    glEnable(GL_DEPTH_TEST);
    //放置一个相机
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(w,h,fu,fv,u0,v0,zNear,zFar),
        pangolin::ModelViewLookAt(posx,posy,posz, target_posx,target_posy,target_posz, pangolin::AxisY)
    );

    //创建视角窗口
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);
    glClearColor(1.0f,1.0f,1.0f,1.0f);                     //白色背景
    pangolin::glDrawAxis(3);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glPointSize(3.0f);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);
    for(auto it : pSystem->landmarks){
        glVertex3f(it[0],it[1],it[2]);
    }
    glEnd();
    pangolin::FinishFrame();

    while( !pangolin::ShouldQuit() )
    {
        usleep(refresh_time);
 //       unique_lock<mutex> lock(vehicle->mrefresh_signal);
        if(pose->isnewone){
        glPointSize(10.0f);
        glBegin(GL_POINTS);

        Vec3f cam_pose =pose->cam_pos;
        Vec3f m_campose=pose->m_campos;
//        glLineWidth(4);
//        glBegin(GL_LINES);
        glColor3f(0.0f,0.0f,0.0f);   //绘制实际位置 黑色
        glVertex3f(cam_pose[0],cam_pose[1],0);

//        glVertex3f(view_length*cos((cam_pose[2]-45.0)*M_PI/180.0),view_length*sin((cam_pose[2]-45.0)*M_PI/180.0),0);		glVertex3f(cam_pose[0],cam_pose[1],0);
//        glVertex3f(view_length*cos((cam_pose[2]+45.0)*M_PI/180.0),view_length*sin((cam_pose[2]+45.0)*M_PI/180.0),0);		glVertex3f(cam_pose[0],cam_pose[1],0);
        glColor3f(0.0f,0.0f,1.0f);   //绘制测量初始状态位置
        glVertex3f(m_campose[0],m_campose[1],0);
//        glVertex3f(view_length*cos((m_campose[2]-45.0)*M_PI/180.0),view_length*sin((m_campose[2]-45.0)*M_PI/180.0),0);		glVertex3f(m_campose[0],m_campose[1],0);
//        glVertex3f(view_length*cos((m_campose[2]+45.0)*M_PI/180.0),view_length*sin((m_campose[2]+45.0)*M_PI/180.0),0);		glVertex3f(m_campose[0],m_campose[1],0);
       pose->isnewone = 0;
       cout<<"测量值"<<pose->m_campos<<endl;
       cout<<"实际值"<<pose->cam_pos<<endl;
       glEnd();
        }
        //交换帧和并推进事件
     pangolin::FinishFrame();
    }

}

Viewer::Viewer(const string &strSettingPath,System *pSys):pSystem(pSys){
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    w = fSettings["observe.w"];
    h = fSettings["observe.h"];
    fu = fSettings["observe.fu"];
    fv = fSettings["observe.fv"];
    u0 = fSettings["observe.u0"];
    v0 = fSettings["observe.v0"];
    zNear = fSettings["observe.zNear"];
    posx = fSettings["observe.posx"];
    posy = fSettings["observe.posy"];
    posz = fSettings["observe.posz"];
    target_posx = fSettings["observe.target_posx"];
    target_posy = fSettings["observe.target_posy"];
    target_posz = fSettings["observe.target_posz"];
    refresh_time = fSettings["sleeptime"];
}

