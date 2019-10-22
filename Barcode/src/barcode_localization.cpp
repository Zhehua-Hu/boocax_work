#include "barcode_localization.h"
int jj =1;

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

        Mat edge;
        Canny(grayImage, edge, 3, 9, 3);
        bitwise_not(edge,edge);
        imshow("edge",edge);
        grayImage = grayImage;
        //二值化
        double thresh_hold;
        threshold(grayImage,grayImage,thresh_hold,255,THRESH_OTSU);
#ifdef Debug
        imshow("二值化灰度图像",grayImage);
        waitKey(0);
#endif

        int N = grayImage.rows;
        vector<int> count_rows(N,0);
        for (int i = 0;i<N; i++ ){
            Scalar temp = sum( grayImage.row(i));
            count_rows[i] = (int)(temp[0]);
        }
        //像素反转
         bitwise_not(grayImage,grayImage);
        imshow("bitwise_not",grayImage);
        Mat bitwise_gray(grayImage);
        //膨胀
        dilate(grayImage, morphImage, getStructuringElement(MORPH_RECT, Size(5,1)),Point(-1,-1),2);
        imshow("膨胀",morphImage);
        //腐蚀
        erode(morphImage, morphImage, getStructuringElement(MORPH_RECT, Size(5,1)),Point(-1,-1),2);
        imshow("先膨胀后腐蚀",morphImage);
#ifdef Debug
        cv::waitKey(0);
#endif

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

             imwrite("picture"+to_string(jj)+".jpg",srcimg2);
                     jj++;

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



bool barcode_veterx_localization(Mat bar_photo, vector<Point2f>& pts2d){
    Mat grayImage;

    if(bar_photo.channels()!=1){
        cvtColor(bar_photo,grayImage,CV_RGB2GRAY);
    }

    int height=grayImage.rows,width=grayImage.cols;

    Point2d left_up,left_down,right_up,right_down;
    Mat lu_img = 255*cv::Mat::ones(cvRound(height*0.6),cvRound(width*0.6),grayImage.type()),ld_img,ru_img,rd_img;

    imshow("LU_ORIGINAL",lu_img);

    ld_img = lu_img.clone();
    rd_img = lu_img.clone();
    ru_img = lu_img.clone();

    cv::Rect rect_lu(0, 0, cvRound(width/2), cvRound(height/2));
    cv::Rect rect_ld(0, cvRound(height/2), cvRound(width/2), cvRound(height/2));
    cv::Rect rect_ru(cvRound(width/2), 0, cvRound(width/2), cvRound(height/2));
    cv::Rect rect_rd(cvRound(width/2), cvRound(height/2), cvRound(width/2), cvRound(height/2));

    grayImage(rect_lu).copyTo(lu_img(Rect(0,0,cvRound(width/2), cvRound(height/2))));
    imshow("LU",lu_img);

    grayImage(rect_ld).copyTo(ld_img(Rect(0,ld_img.rows-cvRound(height/2),cvRound(width/2), cvRound(height/2))));
    imshow("LD",ld_img);

    grayImage(rect_ru).copyTo(ru_img(Rect(cvRound(ru_img.cols-width/2),0,cvRound(width/2), cvRound(height/2))));
    imshow("RU",ru_img);

    grayImage(rect_rd).copyTo(rd_img(Rect(cvRound(rd_img.cols-width/2),cvRound(rd_img.rows-height/2),cvRound(width/2), cvRound(height/2))));
    imshow("RD",rd_img);

    waitKey(0);
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
            diff = abs(srcImg.at<float>(i,j)-temp.at<float>(i,j))<125? 0:1;
            dis=dis + diff*diff;
        }

    }
    return dis;
}

