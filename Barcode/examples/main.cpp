#include "barcode_localization.h"
#if test==0

/*
条形码定位,并用P3P计算出相机位姿程序,流程如下:
读取图像--->二值化--->像素反转--->膨胀--->腐蚀--->寻找最大轮廓--->外包矩形近似--->获取矩形定点--->顶点坐标精确化--->P3P计算位姿
问题: 顶点坐标精确化比较难,目前还没做到这一点
*/

string  image1_path = "/home/oym/boocax_vo/Barcode/Barcode/img/move_all/000088_2019-09-02-16-58-55.jpg";
string  image2_path = "/home/oym/boocax_vo/Barcode/Barcode/img/barcode.jpg";
//Rect roi(146, 238, 114, 38);
Rect roi(80, 215, 140, 50);


int main(){
    Mat srcimg = imread(image1_path),barcode_img;
    srcimg(roi).copyTo(barcode_img);

    imshow("Bodecode",barcode_img);

    waitKey(0);


    int img_width = srcimg.cols,img_height = srcimg.rows;

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
    vector<Point2f> pts2d,pts3d;

    barcode_localization(barcode_img,pts3d);

    waitKey(0);


//    barcode_veterx_localization(srcimg,pts2d);


//     barcode_localization(srcimg,pts3d);
//    Point2f p1(102,97),p2(98,198),p3(203,102),p4(202,199);
//    pts2d.push_back(p1);
//    pts2d.push_back(p2);
//    pts2d.push_back(p3);
//    pts2d.push_back(p4);
//    Mat r,t;
//    solvePnP(points3d,pts2d,K,Mat(), r,t,false,cv::SOLVEPNP_EPNP);
//    Mat R;
//    Rodrigues(r,R);
//    cout<<"t="<<t<<endl;
//    cout<<"R="<<R<<endl;
    waitKey(0);
    return 0;
}

#endif
