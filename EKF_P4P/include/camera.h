#ifndef CAMERA_H
#define CAMERA_H
#include "common.h"
#include <string>
using namespace std;

class System;
class Vehicle;
class Camera{
public:
    float fx_ ,fy_, cx_, cy_ , k1 , k2 , k3, depth_scale_;
    cv::Mat mK;

    cv::Mat  mDistCoef;

    cv::Mat R,t,pos_xyz;

    Vehicle *vehicle;

    std::default_random_engine generator;

    std::normal_distribution<double> dist_uv;//(mean,stddev);

    int N;

    Camera(const string &strSettingPath,System *pSys);

    System *pSystem;

    float stddev ;

    std::vector<cv::Vec3f> landmarks;
    std::vector<cv::Vec2i> uv_landmarks;
    std::vector<cv::Vec2i> m_uvlandmarks;

    std::vector<cv::Point3f> p_landmarks;
    std::vector<cv::Point2f> pm_uvlandmarks;

    void Vec2Point();
    bool see_landmarks();
    cv::Vec2i project_landmark(cv::Vec3f landmark);
    void mearsure_uv();
    void solve_PNP();
    cv::Vec2f pos_xy,posxy_var;
    float yaw,yaw_var;

    int meausure_time=0;

    Camera& operator =(const Camera& cam ){
        if(this !=NULL){
            delete this;
        }
        if(this == &cam)              return *this;
    this->fx_=cam.fx_;    this->fy_=cam.fy_;        this->cx_=cam.cx_;
    this->cy_=cam.cy_;    this->mK = cam.mK.clone(); this->stddev = cam.stddev;
    dist_uv = std::normal_distribution<double>(0, stddev);
    return *this;
    }

    void set_vehicle(Vehicle *v);

};



#endif
