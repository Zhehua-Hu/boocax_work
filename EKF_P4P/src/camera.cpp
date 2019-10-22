#include "camera.h"
ofstream OutFile;

bool Camera::see_landmarks(){
    meausure_time++;
    landmarks.clear();
    uv_landmarks.clear();
    m_uvlandmarks.clear();
    N=0;
    for (auto it:pSystem->landmarks){
        float angle = sin(yaw*M_PI/180)*(it[1]-pos_xy[1])+ cos(yaw*M_PI/180)*(it[0]-pos_xy[0]);
        if (angle <=0) {
            continue;
        }
        if (landmarks.size()==1){            //选的四个点中不能有共线的
            Vec3f d = landmarks[0]- it;
            if (d.dot(d) <0.0001) continue;
        }

        if (landmarks.size()==2){
            if(collineation(landmarks[0],landmarks[1],it)) continue;
        }

        if (landmarks.size()==3){
            if(collineation(landmarks[0],landmarks[1],it)) continue;
            else if(collineation(landmarks[0],landmarks[2],it)) continue;
            else if(collineation(landmarks[1],landmarks[2],it)) continue;
        }

        cv::Vec2i uv = project_landmark(it);
        if(uv[0]>10.0 && uv[0] < 2*cx_-10 && uv[1]>10 && uv[1]<2*cy_-10){
            landmarks.push_back(it);
            uv_landmarks.push_back(uv);
        }
        if (landmarks.size()>=4) break;
    }    
    N=uv_landmarks.size();
    if(N>=4) {
    mearsure_uv();
    Vec2Point();
    solve_PNP();    
    return 1;
    }
    return 0;
}

cv::Vec2i Camera::project_landmark(cv::Vec3f landmark){

    cv::Mat P_landmark = (cv::Mat_<float>(3, 1) << landmark[0],landmark[1], landmark[2]);

    cv::Mat p_cam = R*P_landmark+t;

//    float depth =   p_cam.at<float>(2,0);

//    float x = p_cam.at<float>(0,0);

//    float y = p_cam.at<float>(1,0);

    p_cam = p_cam / p_cam.at<float>(2,0); //归一化

    cv::Mat uvz = mK * p_cam;

    cv::Vec2i uv(uvz.at<float>(0,0),uvz.at<float>(1,0));

    return uv;
}

void Camera::mearsure_uv(){
#if test_PNP
    if(meausure_time == pSystem->step_num/2)     OutFile<<"---实际像素坐标值与测量像素坐标值----"<<endl;
#endif
    for (auto it : uv_landmarks){        
         float m_u = it[0]+dist_uv(generator);
         float m_v = it[1]+dist_uv(generator);
         cv::Vec2i m_uv(m_u,m_v);
#if test_PNP
         if(meausure_time >= pSystem->step_num/2){
         cout<<it<<"-------"<<m_uv<<endl;
         OutFile<<it<<"-------"<<m_uv<<endl;
         }
#endif
         m_uvlandmarks.push_back(m_uv);
    }
}

void Camera::Vec2Point(){
        pm_uvlandmarks.clear();
    for (auto it : m_uvlandmarks){
        cv::Point2f p_temp(it[0],it[1]);
        pm_uvlandmarks.push_back(p_temp);
    }
    p_landmarks.clear();
    for (auto it : landmarks){
        cv::Point3f p_temp(it[0],it[1],it[2]);
        p_landmarks.push_back(p_temp);
    }

}

//给每个测量量一个挠动,以挠动后的结果与未加挠动的结果做比较,近似得出其标准差;
void Camera::solve_PNP(){
    std::vector<cv::Mat> R_vector;
    std::vector<float> yaw_vector;
    std::vector<cv::Mat> t_vector;
    std::vector<cv::Vec2f> p_vector;
    cv::Mat r_,t_,R_,double_t,double_R;
    cv::solvePnP(p_landmarks,pm_uvlandmarks,mK,Mat(),r_,double_t,false,cv::SOLVEPNP_ITERATIVE);
    cv::Rodrigues(r_,double_R);
    R_ = double2float(double_R);
    t_ = double2float(double_t);
    R_vector.push_back(R_);
    t_vector.push_back(t_);
    Vec3f eul = rotationMatrixToEulerAngles(R_);
    float yaw_ = eul[1]*180/M_PI-90;
    check_yaw(yaw_);
    yaw_vector.push_back(yaw_);                        //估计值
    cv::Mat pos_ = -R_.t()*t_;
    cv::Vec2f p_(pos_.at<float>(0,0),pos_.at<float>(1,0));
    p_vector.push_back(p_);
#if Debug
    cout<<"######################"<<endl;
    cout<<"实际的像素坐标为"<<endl;
    for (auto it:uv_landmarks){
        cout<<it<<endl;
    }
    cout<<"观测到的像素坐标为"<<endl;
    for (auto it:pm_uvlandmarks){
        cout<<it<<endl;
    }
    cout<<"实际旋转矩阵"<<R<<endl;
    cout<<"相机测量的旋转矩阵"<<R_<<endl;
    cout<<"实际的欧拉角"<<endl<<rotationMatrixToEulerAngles(R)<<endl;
    cout<<"相机测量得到的欧拉角"<<endl<<eul<<endl;
    cout<<"相机计算得到的位置:"<<endl;

    cout<<"x:"<<p_[0]<<"y:"<<p_[1]<<"yaw"<<yaw_<<endl;
    cout<<"实际位置:"<<endl;
    cout<<"x:"<<vehicle->pos_x_w<<"y:"<<vehicle->pos_y_w<<"yaw:"<<vehicle->yaw<<endl;
    cout<<"########################################################################################"<<endl;
#endif

#if test_PNP
    if(meausure_time>=pSystem->step_num /2){
    OutFile<<"实际旋转矩阵"<<endl<<R<<endl;
    OutFile<<"相机测量的旋转矩阵"<<endl<<R_<<endl;
    OutFile<<"实际的欧拉角"<<endl<<rotationMatrixToEulerAngles(R)<<endl;
    OutFile<<"相机测量的欧拉角"<<endl<<eul<<endl;
    OutFile<<"实际的平移矩阵"<<endl<<t.t()<<endl;
    OutFile<<"相机测量的平移矩阵"<<endl<<t_.t()<<endl;
    OutFile<<"相机计算得到的位置:"<<endl;
    OutFile<<"x:"<<p_[0]<<"; y:"<<p_[1]<<"; yaw:"<<yaw_<<endl;
    OutFile<<"实际位置:"<<endl;
    OutFile<<"x:"<<vehicle->pos_x_w<<"; y:"<<vehicle->pos_y_w<<"; yaw:"<<vehicle->yaw<<endl;
    OutFile<<"########################################################################################"<<endl;
   }
#endif

    std::vector<cv::Point2f> m_uvcopy;
     for (int i=0;i<N;i++){           //每个点均有四种扰动
         m_uvcopy.assign(pm_uvlandmarks.begin(),pm_uvlandmarks.end());
         m_uvcopy[i].x+=stddev;
         cv::Mat double_R1,double_t1,r_1,t_1,R_1;
         cv::solvePnP(p_landmarks,m_uvcopy,mK,Mat(),r_1,double_t1,false,cv::SOLVEPNP_EPNP);
         cv::Rodrigues(r_1,double_R1);
         R_1 = double2float(double_R1);
         t_1 = double2float(double_t1);
         float yaw_1 = rotationMatrixToEulerAngles(R_1)[1]*180/M_PI-90;
         check_yaw(yaw_1);
         R_vector.push_back(R_1);
         t_vector.push_back(t_1);
         yaw_vector.push_back(yaw_1);
         cv::Mat pos_1 = -R_1.t()*t_1;
         cv::Vec2f p_1(pos_1.at<float>(0,0),pos_1.at<float>(1,0));
         p_vector.push_back(p_1);

         m_uvcopy.clear();
         m_uvcopy.assign(pm_uvlandmarks.begin(),pm_uvlandmarks.end());
         m_uvcopy[i].x-=stddev;
         cv::Mat double_R2,double_t2,r_2,t_2,R_2;
         cv::solvePnP(p_landmarks,m_uvcopy,mK,Mat(),r_2,double_t2,false,cv::SOLVEPNP_EPNP);
         cv::Rodrigues(r_2,double_R2);
         R_2 = double2float(double_R2);
         t_2 = double2float(double_t2);
         float yaw_2 = rotationMatrixToEulerAngles(R_2)[1]*180/M_PI-90;
         check_yaw(yaw_2);
         R_vector.push_back(R_2);
         t_vector.push_back(t_2);
         yaw_vector.push_back(yaw_2);
         cv::Mat pos_2 = -R_2.t()*t_2;
         cv::Vec2f p_2(pos_2.at<float>(0,0),pos_2.at<float>(1,0));
         p_vector.push_back(p_2);

         m_uvcopy.clear();
         m_uvcopy.assign(pm_uvlandmarks.begin(),pm_uvlandmarks.end());
         m_uvcopy[i].y+=stddev;
         cv::Mat double_R3,double_t3,r_3,t_3,R_3;
         cv::solvePnP(p_landmarks,m_uvcopy,mK,Mat(),r_3,double_t3,false,cv::SOLVEPNP_EPNP);
         cv::Rodrigues(r_3,double_R3);
         R_3 = double2float(double_R3);
         t_3 = double2float(double_t3);
         float yaw_3 = rotationMatrixToEulerAngles(R_3)[1]*180/M_PI-90;
         check_yaw(yaw_3);
         R_vector.push_back(R_3);
         t_vector.push_back(t_3);
         yaw_vector.push_back(yaw_3);
         cv::Mat pos_3 = -R_3.t()*t_3;
         cv::Vec2f p_3(pos_3.at<float>(0,0),pos_3.at<float>(1,0));
         p_vector.push_back(p_3);

         m_uvcopy.clear();
         m_uvcopy.assign(pm_uvlandmarks.begin(),pm_uvlandmarks.end());
         m_uvcopy[i].y-=stddev;
         cv::Mat double_R4,double_t4,r_4,t_4,R_4;
         cv::solvePnP(p_landmarks,m_uvcopy,mK,Mat(),r_4,double_t4,false,cv::SOLVEPNP_EPNP);
         cv::Rodrigues(r_4,double_R4);
         R_4 = double2float(double_R4);
         t_4 = double2float(double_t4);
         float yaw_4 = rotationMatrixToEulerAngles(R_4)[1]*180/M_PI-90;
         check_yaw(yaw_4);
         R_vector.push_back(R_4);
         t_vector.push_back(t_4);
         yaw_vector.push_back(yaw_4);
         cv::Mat pos_4 = -R_4.t()*t_4;
         cv::Vec2f p_4(pos_4.at<float>(0,0),pos_4.at<float>(1,0));
         p_vector.push_back(p_4);
     }
      posxy_var = cal_variance(p_vector);
      yaw_var   = cal_variance(yaw_vector);
      pos_xy = p_vector[0];
      yaw    = yaw_vector[0];
      check_yaw(yaw);
}


Camera::Camera(const string &strSettingPath,System *pSys):pSystem(pSys)
         {
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
     fx_ = fSettings["Camera.fx"];
     fy_ = fSettings["Camera.fy"];
     cx_ = fSettings["Camera.cx"];
     cy_ = fSettings["Camera.cy"];
     stddev = fSettings["Camera.stddev"];
     cv::Mat K = cv::Mat::eye(3,3,CV_32F);
     K.at<float>(0,0) = fx_;
     K.at<float>(1,1) = fy_;
     K.at<float>(0,2) = cx_;
     K.at<float>(1,2) = cy_;
     K.copyTo(mK);
     dist_uv = std::normal_distribution<double>(0, stddev);
        }

void Camera::set_vehicle(Vehicle *v){
    vehicle = v;
    R=v->R.clone();
    cv::Mat pos = cv::Mat::eye(3,1,CV_32F);
    pos.at<float>(0,0) = v->pos_x_w;
    pos.at<float>(1,0) = v->pos_y_w;
    pos.at<float>(2,0) = 1;
    pos_xyz = pos.clone();
    pos_xy[0] = v->pos_x_w;
    pos_xy[1] = v->pos_y_w;
    t=-R*pos_xyz;
}



