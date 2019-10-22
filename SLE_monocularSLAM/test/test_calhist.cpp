#include "../include/common.h"
#if TEST_CALHIST
int main(int argc,char*argv[])
{
    Mat image = imread("/home/oym/boocax_vo/pose_Of_TableLeg/photo_Of_tableLeg/000000_2019-05-14-10-05-13.jpg", 0);
    //【2】检查是否载入成功
    if (image.empty())
    {
        printf("读取图片错误，请确认目录下是否有imread函数指定图片存在！ \n ");
        return 0;
    }
    //【3】计算灰度图像一维直方图
    const int nimages = 1;
    int channels[] = {0};
    MatND hist;
    int dims = 1;
    int gap = 10;
    int histSize[] = {cvRound(256/gap)};
    float hranges[] = {0.0, 255.0};
    const float* ranges[] = {hranges};//一定要有const
    calcHist(&image, nimages, channels, Mat(), hist, dims, histSize, ranges);
    //【4】获取最大最小值
    double minValue = 0;
    double maxValue = 0;
    minMaxLoc(hist, &minValue, &maxValue);
    //【5】绘制灰度图像一维直方图
    Mat dstImage(256, 256, CV_8U, Scalar(0));//256*256的黑色底板
    int hpt = saturate_cast<int>(0.9 * 256);
    for (int i = 0; i < cvRound(256/gap); i++)
    {
        float binValue = hist.at<float>(i);
        //统计数值的缩放，增强直方图的可视化
        int realValue = saturate_cast<int>(binValue * hpt / maxValue);
        //在256*256的黑色底板上画矩形
        rectangle(dstImage, Point(i*gap, 256), Point((i+1)*gap, 256-realValue), Scalar(255));
    }
    //【6】显示图像
    imshow("21-灰度原图", image);
    imshow("21-一维直方图", dstImage);
    //【7】保持窗口显示
    waitKey(0);
    return 0;

}


#endif
