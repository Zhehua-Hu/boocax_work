#ifndef VIEWER_H
#define VIEWER_H
#include <pangolin/pangolin.h>
#include <opencv2/core.hpp>
#include "common.h"
using namespace cv;
using namespace std;
class Vehicle;
class System;

class Viewer
{
public:
    System* pSystem;
    Viewer(const string &strSettingPath,System *pSys);
    void Run();   //初始化将路标点和坐标系绘制出来
    float view_length=0.2;
    Vehicle *vehicle;
    new_pose* pose;
    vector<Vec3f> landmarks;
    void setVehicle(Vehicle *v);
    void setlandmarks(vector<Vec3f> land_marks);
    int w, h;
    pangolin::GLprecision fu, fv, u0, v0, zNear, zFar;
    float posx,posy,posz,target_posx,target_posy,target_posz;
    float refresh_time;
};
#endif // VIEWER_H
