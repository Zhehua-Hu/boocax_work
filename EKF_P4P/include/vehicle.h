#ifndef VEHICLE_H
#define VEHICLE_H
#include <random>
#include <iostream>
#include <math.h>
#include <vector>
#include "camera.h"
#include <Eigen/Eigen>
#include <Eigen/Core>
#include "common.h"
#include <mutex>
#include <thread>

using namespace std;
using namespace  Eigen;
class System;
class Camera;
struct new_pose;
class Vehicle
{
public:
    Vehicle(const string &strSettingPath);
    float system_time = 0;
    float posx_stddev,posy_stddev,yaw_stddev,yawspeed_stddev;
    std::default_random_engine generator;
    std::normal_distribution<double> dist_v;//(0, 0.1*dt);
    std::normal_distribution<double> dist_w;//(0, 10*dt);
    float speed_x_w,speed_y_w,pos_x_w,pos_y_w,yaw,yaw_speed,roll,pitch;  //世界坐标系
    float r_l,r_right;  //车轮半径
    float w_l,w_r;      //车轮速度
    float speed_b,yawspeed_b,speed_stddev;   //本体速度及测量标准差
    float m_speedx_w,m_speedy_w,m_speed_b,m_yawspeed,m_posx_w,m_posy_w,m_yaw; //测量值
    float dt;  //间隔时间
    Camera *cam;
    void move_wheels(float _wl,float _wr);
    void move_bodyspeed(float v,float v_vertical,float w);   //实际本体行动速度
    bool observe_camera();
    void copy(Vehicle vehicle);
    new_pose *pose;
    void refresh_lastpose();
    std::vector<cv::Vec3f> landmarks;
    std::mutex mrefresh_signal;
    cv::Mat R;
};



#endif // VEHICLE_H
