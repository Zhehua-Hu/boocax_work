#include "barcode_localization.h"

bool barcode_localization(Mat srcImage, vector<Point2f>& pts2d)
{
    Mat grayImage,blurImage,thresholdImage,gradientXImage,gradientYImage,gradientImage,morphImage;
    {
        if(srcImage.empty())
        {
            cout<<"image file read error"<<endl;

            return -1;
        }
        //图像转换为灰度图像
        if(srcImage.channels() == 3)
        {
            cvtColor(srcImage,grayImage,CV_RGB2GRAY);
        }
        else
        {
            grayImage = srcImage.clone();
        }
        Mat src_img=grayImage.clone();
        imshow("原图",grayImage);
        //建立图像的梯度幅值
        threshold(grayImage,grayImage,100,255,THRESH_BINARY);
        imshow("grayImage",grayImage);
        bitwise_not(grayImage,grayImage);
        imshow("bitwise_not",grayImage);
        Mat bitwise_gray (grayImage);
        Scharr(grayImage,gradientXImage,CV_32F,1,0);
        Scharr(grayImage,gradientYImage,CV_32F,0,1);
        //因为我们需要的条形码在需要X方向水平,所以更多的关注X方向的梯度幅值,而省略掉Y方向的梯度幅值
        subtract(gradientXImage,gradientYImage,gradientImage);
        //归一化为八位图像
        convertScaleAbs(gradientImage,gradientImage);
        //看看得到的梯度图像是什么样子
        imshow("gradientImage",gradientImage);
        //对图片进行相应的模糊化,使一些噪点消除
         blur(gradientImage,blurImage,Size(3,3));
         imshow("blurImage",blurImage);

        //模糊化以后进行阈值化,得到到对应的黑白二值化图像,二值化的阈值可以根据实际情况调整
        threshold(blurImage,thresholdImage,210,255,THRESH_BINARY);
        //看看二值化图像
         imshow("thresholdImage",thresholdImage);
        //二值化以后的图像,条形码之间的黑白没有连接起来,就要进行形态学运算,消除缝隙,相当于小型的黑洞,选择闭运算
        //因为是长条之间的缝隙,所以需要选择宽度大于长度
        Mat kernel = getStructuringElement(MORPH_RECT,Size(21,7));
        namedWindow("kernel",WINDOW_NORMAL);
        imshow("kernel",kernel);

//        morphologyEx(grayImage,morphImage,MORPH_CLOSE,kernel);
        //看看形态学操作以后的图像
//        imshow("morphologyEx",morphImage);
        //现在要让条形码区域连接在一起,所以选择膨胀腐蚀,而且为了保持图形大小基本不变,应该使用相同次数的膨胀腐蚀
        //先腐蚀,让其他区域的亮的地方变少最好是消除,然后膨胀回来,消除干扰,迭代次数根据实际情况选择
        erode(grayImage, morphImage, getStructuringElement(MORPH_RECT, Size(21,7)),Point(-1,-1),1);
        imshow("腐蚀",morphImage);
        dilate(morphImage, morphImage, getStructuringElement(MORPH_RECT, Size(21,7)),Point(-1,-1),1);
        imshow("先腐蚀后膨胀",morphImage);


        dilate(grayImage, morphImage, getStructuringElement(MORPH_RECT, Size(21,7)),Point(-1,-1),1);
        imshow("膨胀",morphImage);
        erode(morphImage, morphImage, getStructuringElement(MORPH_RECT, Size(21,7)),Point(-1,-1),1);
        imshow("先膨胀后腐蚀",morphImage);

//        dilate(morphImage, morphImage, getStructuringElement(MORPH_RECT, Size(3,3)),Point(-1,-1),3);
//        //看看形态学操作以后的图像
//        imshow("腐蚀后膨胀",morphImage);
        vector<vector<Point2i>>contours;
        vector<float>contourArea;
        //接下来对目标轮廓进行查找,目标是为了计算图像面积
        findContours(morphImage,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);

        if(contours.size() == 0)   return 0;
        //计算轮廓的面积并且存放
        for(int i = 0; i < contours.size();i++)
        {
            contourArea.push_back(cv::contourArea(contours[i]));
        }
        //找出面积最大的轮廓
        double maxValue;
        Point maxLoc;
        minMaxLoc(contourArea, NULL,&maxValue,NULL,&maxLoc);
        //计算面积最大的轮廓的最小的外包矩形
        RotatedRect minRect = minAreaRect(contours[maxLoc.x]);
        //为了防止找错,要检查这个矩形的偏斜角度不能超标
        //如果超标,那就是没找到
        if(minRect.angle<2.0)
        {
            //找到了矩形的角度,但是这是一个旋转矩形,所以还要重新获得一个外包最小矩形
            Rect myRect = boundingRect(contours[maxLoc.x]);
            //把这个矩形在源图像中画出来
            rectangle(bitwise_gray,myRect,Scalar(255,255,255),3,LINE_AA);
            //看看显示效果,找的对不对
            imshow("框图",bitwise_gray);
            //将扫描的图像裁剪下来,并保存为相应的结果,保留一些X方向的边界,所以对rect进行一定的扩张
//             myRect.x= myRect.x - (myRect.width/20);
//             myRect.width = myRect.width*1.1;
 //          Mat resultImage = Mat(srcImage,myRect);
//           imshow("barcode",resultImage);
             Point2i left_up(myRect.x,myRect.y);
             Point2i left_down(myRect.x,myRect.y+myRect.height);
             Point2i right_up(myRect.x+myRect.width,myRect.y);
             Point2i right_down(myRect.x+myRect.width,myRect.y+myRect.height);
             pts2d.push_back(left_up);
             pts2d.push_back(left_down);
             pts2d.push_back(right_up);
             pts2d.push_back(right_down);
             Mat srcimg2 = src_img.clone();
             for (Point2f pt: pts2d){
                 circle (srcimg2, pt ,3,255,1.5);
             }
             imshow("矩形顶点",srcimg2);

             precise_localization(srcImage,pts2d,10);

             for (Point2f pt: pts2d){
                 circle (src_img, pt ,3,255,1.5);
             }
             imshow("精确顶点",src_img);


        }
        else
        {
            return 0;
        }
    }
    return 1;
}



void precise_localization(Mat srcImage, vector<Point2f>& pt , int radius){
 Mat temp_leftup = (Mat_<float>(3,3)<<255,255,255,255,0,0,255,0,0);
 Mat temp_leftdown = (Mat_<float>(3,3)<<255,0,0,255,0,0,255,255,255);
 Mat temp_rightup = (Mat_<float>(3,3)<<255,255,255,0,0,255,0,0,255);
 Mat temp_rightdown = (Mat_<float>(3,3)<<0,0,255,0,0,255,255,255,255);
 vector<Mat> temp;
 temp.push_back(temp_leftup);
 temp.push_back(temp_leftdown);
 temp.push_back(temp_rightup);
 temp.push_back(temp_rightdown);
 Mat image;
 srcImage.convertTo(image,CV_32F);
 vector<Point2f> points2d ;
 points2d.assign(pt.begin(),pt.end());
 pt.clear();
 for (int i =0 ; i<4; i++){
     Mat Temp = temp[i];
     Point2f pt2d(points2d[i]);
     Point2f precise_point;
     float min_dis = 9;
     float dis = 0;
     for (int j= -radius;j<radius;j++){
         for(int k =-radius;k<radius;k++){
             Rect rect(pt2d.x+j-1, pt2d.y+k-1, 3, 3);
             dis = cal_distance(image(rect),Temp);
             if (dis<min_dis){
                 min_dis = dis;
                 precise_point.x=pt2d.x+j;
                 precise_point.y=pt2d.y+k;
                              }
         }
     }
     pt.push_back(precise_point);
 }

}

float cal_distance(const Mat srcImg,const Mat temp){
    float dis=0,diff=0;
    if(srcImg.cols!= temp.cols || srcImg.rows!= temp.rows ) return 100000;
    for (int i = 0; i<temp.rows; i++){
        for (int j = 0; j<temp.cols; j++){
            diff = abs(srcImg.at<float>(i,j)-temp.at<float>(i,j))<150? 0:1;
            dis=dis + diff*diff;
        }

    }
    return dis;
}

