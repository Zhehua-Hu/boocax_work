/*本程序计算两帧图像之间的位姿变换
 输入:两帧图像
 输出:图像之间的位姿矩阵
 算法流程:输入图像--->提取线段端点--->计算sift描述子--->描述子匹配--->匹配点坐标去畸变
               --->RANSAC算法计算F和H矩阵并选择合适的矩阵模型------>恢复R,t;
*/


#include "include/visual_odometry.h"
#if TEST_PICTURE
int main(int argc, char *argv[])
{
    string file_dir = "default.yaml";
    SLE_monocularSLAM::Config::setParameterFile ( file_dir);
    SLE_monocularSLAM::VisualOdometry::Ptr vo ( new SLE_monocularSLAM::VisualOdometry );
    SLE_monocularSLAM::Camera::Ptr camera ( new SLE_monocularSLAM::Camera );
    vo->camera_=camera;
    Mat srcimg3 = cv::imread("/home/oym/SLE_monocularSLAM/image/3.png",IMREAD_GRAYSCALE);
    Mat srcimg2 = cv::imread("/home/oym/SLE_monocularSLAM/image/2.png",IMREAD_GRAYSCALE);   //图像目录需要修改到自己的目录
    SLE_monocularSLAM::Frame::Ptr pFrame = SLE_monocularSLAM::Frame::createFrame();
    pFrame->camera_ = camera;
    pFrame->gray_ = srcimg2;
    vo->addFrame ( pFrame );
    pFrame->gray_ = srcimg3;
    vo->addFrame ( pFrame );
   while(1){
     char key =  waitKey(0);
     if(key== ' ') break;
   }
    return 0;
}
#endif
