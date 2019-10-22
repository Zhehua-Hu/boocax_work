#ifndef SYSTEM_H
#define SYSTEM_H
#include  <thread>
#include "viewer.h"
#include "common.h"
#include "camera.h"
class Camera;
class Vehicle;
class Viewer;
struct new_pose;
class System
{
public:
    System(std::vector<cv::Vec3f> land_marks, const string &strSettingsFile);

    Vehicle* AGV;

    Camera* cam;

    std::vector<cv::Vec3f> landmarks;

    std::thread* mptViewer;

    Viewer* view;

    void run(float v,float v_vertical,float w);

    new_pose* pose ;

    float sleeptime;   //系统运行周期

    int step_num;     //系统运行步数
    };




#endif // SYSTEM_H
