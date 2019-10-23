#include "system.h"
//EKN-PNP算法仿真.小车在二维平面运动,状态量(x,y,yaw)有一个真实的速度,产生真实轨迹,测量有噪声的速度,可以递推下一时刻的位置及其标准差;
//看到4个及以上路标点时,可用PNP算法求得一个位置及标准差,两者融合,得到融合后的位置及标准差;
//计算频率15Hz;
//PNP:路标点在相机会产生投影,加一定标准差的噪声(0,5),得到测量值,以测量值解算出来的位姿作为相机计算位姿均值,给每个像素点一以测量值;
//标准差的扰动,计算出挠动后的位置信息,与测量像素算出来的位置进行计算得到标准差;
//系统运行输入:机体坐标系的前进速度和绕机体运动的角速度;
//位置求解输入:测量得到的速度,角速度和路标正投影点的像素;
//没考虑相机畸变;
//初始版基于机体速度和角速度,真实情况是小车轮子速度通过正运动学解算得到机体运动速度的;
//单独线程使用pangolin进行画图;

System::System(std::vector<cv::Vec3f> land_marks,const string &strSettingsFile):landmarks(land_marks){

cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);  //strSettingsFile.c_str()就是指strSettingsFile字符串的内容
if(!fsSettings.isOpened())
{
  cerr << "Failed to open settings file at: " << strSettingsFile << endl;
  exit(-1);
}
sleeptime = fsSettings["sleeptime"];
step_num  = fsSettings["step_num"];
cam = new Camera(strSettingsFile,this);
AGV = new Vehicle(strSettingsFile);
AGV->cam = cam;
cam->set_vehicle(AGV);
view = new Viewer(strSettingsFile,this);
view->setVehicle(AGV);
view->pose    = AGV->pose;
mptViewer = new thread(&Viewer::Run,view);
sleep(2);
}

void System::run(float v_forward,float v_verticle,float w)
{
    int i=0;
#if test_PNP
    while(i<step_num){
        if(i<step_num/2)    AGV->move_bodyspeed(v_forward,v_verticle,w);
        else AGV->move_bodyspeed(0,0,0);
     usleep(sleeptime);
    i++;
    }
#else
    while(i<step_num){
    AGV->move_bodyspeed(v_forward,v_verticle,w);
     usleep(sleeptime);
    i++;
    }
#endif
    cv::waitKey(0);
}


