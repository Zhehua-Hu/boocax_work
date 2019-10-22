#include "barcode_localization.h"
#include "solve_pnp.h"
#if test==1
/*
条形码定位,并用P3P计算出相机位姿程序,流程如下:
读取图像--->二值化--->像素反转--->膨胀--->腐蚀--->寻找最大轮廓--->外包矩形近似--->获取矩形定点--->顶点坐标精确化--->P3P计算位姿
问题: 顶点坐标精确化比较难,目前还没做到这一点
*/
int main(){
    float barcode_width,barcode_heigth;
    Mat K = (Mat_<double>(3,3)<<520,0,325,0,520,250,0,0,1);
    barcode_width=10;
    barcode_heigth=3;
    vector<Point3f> points3d;
    Point3f left_up(0,barcode_heigth,0);
    points3d.push_back(left_up);
    Point3f left_down(0,0,0);
    points3d.push_back(left_down);
    Point3f right_up(barcode_width,barcode_heigth,0);
    points3d.push_back(right_up);
    Point3f right_down(barcode_width,0,0);
    points3d.push_back(right_down);
    vector<Point2f> pts2d,real_pts2d;

    Point2f p1(97,103),p2(103,153),p3(248,103),p4(252,152); //real position p1(100,100),p2(100,200),p3(200,100),p4(200,200)
    pts2d.push_back(p1);
    pts2d.push_back(p2);
    pts2d.push_back(p3);
    pts2d.push_back(p4);
    Mat r,t;
    solvePnP(points3d,pts2d,K,Mat(), r,t,false,cv::SOLVEPNP_EPNP);
    Mat R;
    Rodrigues(r,R);
    cout<<"t="<<t<<endl;
    cout<<"R="<<R<<endl;
    cout<<"bias_amera_center:"<<endl<<-R.t()*t<<endl;
    cout<<"==================================="<<endl;
    waitKey(0);

    Point2f real_p1(100,100),real_p2(100,150),real_p3(250,100),real_p4(250,150);
    real_pts2d.push_back(real_p1);
    real_pts2d.push_back(real_p2);
    real_pts2d.push_back(real_p3);
    real_pts2d.push_back(real_p4);

    Mat real_r,real_t,real_R;
    solvePnP(points3d,real_pts2d,K,Mat(), real_r,real_t,false,cv::SOLVEPNP_EPNP);
    Rodrigues(real_r,real_R);
    cout<<"t="<<real_t<<endl;
    cout<<"R="<<real_R<<endl;
    cout<<"real_camera_center:"<<endl<<-real_R.t()*real_t<<endl;
    cout<<"==================================="<<endl;

    solve_pnp(points3d,pts2d,K,Mat(),false,cv::SOLVEPNP_EPNP,2);

    return 0;
}

#endif
