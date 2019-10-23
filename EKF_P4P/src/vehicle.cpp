#include "vehicle.h"

void Vehicle::move_bodyspeed(float v,float v_verticla,float w){
//实际状态变换
    float v_lateral = v_verticla;
    system_time +=dt;
    speed_b=v;
    yawspeed_b=w;
    float radian_yaw = yaw*M_PI/180.0;
    yaw_speed = yawspeed_b;
    speed_x_w = v * cos(radian_yaw);
    speed_y_w = v*sin(radian_yaw);
    yaw += yaw_speed*dt;
    check_yaw(yaw);
    pos_x_w += speed_x_w*dt+v_lateral*dt*cos((yaw+90)*M_PI/180.0);
    pos_y_w += speed_y_w*dt+v_lateral*dt*sin((yaw+90)*M_PI/180.0);
//测量状态变换
    float m_radian_yaw = m_yaw *M_PI/180.0;
    yaw_stddev = sqrt(yaw_stddev*yaw_stddev + dt*dt*yawspeed_stddev*yawspeed_stddev);   //下一时刻角度标准差
    posx_stddev = sqrt(posx_stddev*posx_stddev + speed_stddev*speed_stddev*dt*dt*cos(m_radian_yaw)*cos(m_radian_yaw));
    posy_stddev = sqrt(posy_stddev*posy_stddev + speed_stddev*speed_stddev*dt*dt*sin(m_radian_yaw)*sin(m_radian_yaw));
    m_speed_b = v + dist_v(generator);
    float m_vlateral = v_verticla+dist_v(generator);
    m_yawspeed = w + dist_w(generator);
    m_speedx_w = m_speed_b * cos(m_radian_yaw);
    m_speedy_w = m_speed_b * sin(m_radian_yaw);
    m_yaw += m_yawspeed*dt;
    check_yaw(m_yaw);
    m_posx_w += m_speedx_w*dt+m_vlateral*dt*cos((m_yaw+90)*M_PI/180.0);
    m_posy_w += m_speedy_w*dt+m_vlateral*dt*sin((m_yaw+90)*M_PI/180.0);
    if(observe_camera()){
        m_posx_w =(m_posx_w * cam->posxy_var[0]+cam->pos_xy[0]*posx_stddev*posx_stddev)/(cam->posxy_var[0]+posx_stddev*posx_stddev);
        m_posy_w = (m_posy_w * cam->posxy_var[1]+cam->pos_xy[1]*posy_stddev*posy_stddev)/(cam->posxy_var[1]+posy_stddev*posy_stddev);
        m_yaw = (m_yaw *cam->yaw_var + cam->yaw*yaw_stddev*yaw_stddev)/(cam->yaw_var+yaw_stddev*yaw_stddev);
        posx_stddev = sqrt(posx_stddev*posx_stddev*cam->posxy_var[0]/(cam->posxy_var[0]+posx_stddev*posx_stddev));
        posy_stddev = sqrt(posy_stddev*posy_stddev*cam->posxy_var[1]/(cam->posxy_var[1]+posy_stddev*posy_stddev));
        yaw_stddev = sqrt(yaw_stddev*yaw_stddev*cam->yaw_var/(yaw_stddev*yaw_stddev+cam->yaw_var));
    }
    unique_lock<mutex> lock(mrefresh_signal);
    refresh_lastpose();
}

bool Vehicle::observe_camera(){
    cv::Vec3f euler_angle(-90*M_PI/180.0,(90+yaw)*M_PI/180.0,0.0); //X-Y-Z
    cv::Mat Rot = eulerAnglesToRotationMatrix(euler_angle).clone();
    Rot.copyTo(cam->R);
    cv::Mat pos = (cv::Mat_<float>(3, 1) << pos_x_w,pos_y_w, 1);
    pos.copyTo(cam->pos_xyz);
    cam->t = -cam->R * cam->pos_xyz;
    cam->yaw = yaw;
    if(cam->see_landmarks()){
        cout<<"see landmarks"<<endl;
        return 1;   //看路标点
    }
    return 0;
}

void Vehicle::copy(Vehicle vehicle){
    this->system_time=vehicle.system_time;
    this->posx_stddev=vehicle.posx_stddev;
    this->posy_stddev=vehicle.posx_stddev;
    this->yaw_stddev=vehicle.yaw_stddev;
    this->yawspeed_stddev=vehicle.yawspeed_stddev;
    this->dist_v=vehicle.dist_v;
    this->dist_w=vehicle.dist_w;
    this->m_posx_w=vehicle.m_posx_w;
    this->m_posy_w=vehicle.m_posy_w;
    this->m_yaw=vehicle.m_yaw;
    this->dt=vehicle.dt;
    this->roll=vehicle.roll,
    this->pitch=vehicle.pitch;
    std::normal_distribution<double> dist_pos(0, posx_stddev);
    std::normal_distribution<double> dist_yaw(0, yaw_stddev);  //角度单位为度
    this->pos_x_w = m_posx_w + dist_pos(generator);
    this->pos_y_w = m_posy_w + dist_pos(generator);
    this->yaw = m_yaw+dist_yaw(generator);
    this->cam = vehicle.cam;
//        dist_v = std::normal_distribution<double>(0, 0.1*dt);
//        dist_w = std::normal_distribution<double>(0, yawspeed_stddev*dt);

}

void Vehicle::refresh_lastpose(){
    cout<<pos_x_w<<pos_y_w<<yaw<<endl;
    pose->cam_pos = cv::Vec3f(pos_x_w,pos_y_w,yaw);
    pose->m_campos = cv::Vec3f(m_posx_w,m_posy_w,m_yaw);
    pose->isnewone = 1;

}

Vehicle::Vehicle(const string &strSettingPath):m_posx_w(0),m_posy_w(0),m_yaw(0),roll(0.0),pitch(0.0),
m_yawspeed(0),m_speed_b(0),speed_b(0),yaw_speed(0){
       pose = new new_pose;
       cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
       posx_stddev = fSettings["posx_stddev"];
       posy_stddev = fSettings["posy_stddev"];
       yaw_stddev  = fSettings["yaw_stddev"];
       yawspeed_stddev = fSettings["yawspeed_stddev"];
       speed_stddev = fSettings["speed_stddev"];
       dt = fSettings["dt"];
       std::normal_distribution<double> dist_pos(0, posx_stddev);
       std::normal_distribution<double> dist_yaw(0, yaw_stddev);  //角度单位为度
       pos_x_w = m_posx_w + dist_pos(generator);
       pos_y_w = m_posy_w + dist_pos(generator);
       yaw = m_yaw+dist_yaw(generator);
       dist_v = std::normal_distribution<double>(0, speed_stddev);
       dist_w = std::normal_distribution<double>(0, yawspeed_stddev);
       R = eulerAnglesToRotationMatrix(Vec3f(0,0,yaw)).clone();
       refresh_lastpose();
}
