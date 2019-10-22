#include "../include/common.h"
#if TEST_HARRIES
#include "core/core.hpp"
#include "highgui/highgui.hpp"
#include "imgproc/imgproc.hpp"
using namespace cv;
using namespace std;
Mat image;
Mat imageGray;
int thresh=200;
int MaxThresh=255;
void Trackbar(int,void*);  //阈值控制
int main(int argc,char*argv[])
{
    image=imread("/home/oym/boocax_vo/pose_Of_TableLeg/photo_Of_tableLeg/000000_2019-05-14-10-05-13.jpg");
    cvtColor(image,imageGray,CV_RGB2GRAY);
    GaussianBlur(imageGray,imageGray,Size(5,5),1); // 滤波
    namedWindow("Corner Detected");
    createTrackbar("threshold：","Corner Detected",&thresh,MaxThresh,Trackbar);
    imshow("Corner Detected",image);
    Trackbar(0,0);
    waitKey(0);
    return 0;
}

void Trackbar(int,void*)
{
    Mat dst,dst8u,dstshow,imageSource;
    dst=Mat::zeros(image.size(),CV_32FC1);
    imageSource=image.clone();
    cornerHarris(imageGray,dst,3,3,0.04,BORDER_DEFAULT);
    normalize(dst,dst8u,0,255,CV_MINMAX);  //归一化
    convertScaleAbs(dst8u,dstshow);
    imshow("dst",dstshow);  //dst显示
    for(int i=0;i<image.rows;i++)
    {
        for(int j=0;j<image.cols;j++)
        {
            if(dstshow.at<uchar>(i,j)>thresh)  //阈值判断
            {
                circle(imageSource,Point(j,i),2,Scalar(0,0,255),2); //标注角点
            }
        }
    }
    imshow("Corner Detected",imageSource);
}
#endif
