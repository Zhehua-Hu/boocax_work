#include"../include/common.h"
#if FINDOUTLINE
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include "../3rd_party/descriptor_custom.hpp"
using namespace std;
using namespace cv;
float width = 1280;
float height = 720;
struct leg_position{
    Point2f center;
    float width;
    float height;
};

void findoutline(const Mat image){
    Mat img = image.clone();
    RNG rng(12345);

    cvtColor( img, img, CV_BGR2GRAY );
    blur( img, img, Size(3,3) );
    Mat canny_output, threshold_output;
    vector<vector<Point> > contours,contours_2;
    vector<Vec4i> hierarchy,hierarchy2;
    int thresh = 100/5;
      /// 用Canny算子检测边缘,做轮廓检测
    Canny( img, canny_output, thresh, thresh*2, 3 );
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
       {
         Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
         drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
       }
    /// 在窗体中显示结果



// 二值化再做轮廓检测
      double t;
      t=threshold(img, threshold_output, 170, 255, CV_THRESH_BINARY | THRESH_OTSU);

      cout<<endl<<"阈值为:"<<t<<endl;
      /// 寻找轮廓
      findContours( threshold_output, contours_2, hierarchy2, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
      /// 绘出轮廓
      Mat drawing2 = Mat::zeros( threshold_output.size(), CV_8UC3 );
      for( int i = 0; i< contours_2.size(); i++ )
         {
           Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
           drawContours( drawing2, contours_2, i, color, 2, 8, hierarchy2, 0, Point() );
         }
      /// 在窗体中显示结果
      namedWindow( "Source_image", CV_WINDOW_AUTOSIZE );
      imshow( "Source_image", img );
      moveWindow("Source_image",img.cols*5,200);

      namedWindow( "threshold_output", CV_WINDOW_AUTOSIZE );
      imshow( "threshold_output", threshold_output );
      moveWindow("threshold_output",img.cols*11,100);

      namedWindow( "threshold_outline", CV_WINDOW_AUTOSIZE );
      imshow( "threshold_outline", drawing2 );
      moveWindow("threshold_outline",img.cols*17,100);


      namedWindow( "Canny", CV_WINDOW_AUTOSIZE );
      imshow( "Canny", canny_output );
      moveWindow("Canny",img.cols*11,500);

      namedWindow( "CannyContours", CV_WINDOW_AUTOSIZE );
      imshow( "CannyContours", drawing );
      moveWindow("CannyContours",img.cols*17,500);

      waitKey(0);
    }

void handletable(leg_position l, Mat img ){
    cv::Mat image = img.clone() ;
    Mat ROI;
    Point2f up_left(l.center.x-l.width/2,l.center.y-l.height/2);
    up_left.x = max(1.0f,up_left.x);
    up_left.y = max(1.0f,up_left.y);
    Rect rect(up_left.x,up_left.y, l.width, l.height);
    ROI = image(rect);
    findoutline(ROI);
    waitKey(0);
}


vector<string> getFiles(string cate_dir)
{
    vector<string> files,files_jpg;//存放文件名
    DIR *dir; struct
    dirent *ptr; char base[1000];
    if ((dir=opendir(cate_dir.c_str())) == NULL)
        {
        perror("Open dir error...");
        exit(1);
        }
    while ((ptr=readdir(dir)) != NULL)
    {
        cout<<ptr->d_name<<endl;
        cout<<sizeof(ptr->d_name)/8<<endl;
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)
         continue;
        else if(ptr->d_type == 8) ///file //
        files.push_back(ptr->d_name);
        else if(ptr->d_type == 10) ///link file //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            continue;
        else if(ptr->d_type == 4) ///dir
        {
            files.push_back(ptr->d_name);
               }
    }
    closedir(dir);
    string temp;
    for (string s: files ){
        if (s[s.length()-1] == 'g'){
            cout<<s<<endl;
            temp = s.substr(0,s.length()-4);
            files_jpg.push_back(temp);
        }
    }
    return files_jpg;
}

void readTxt(string file , vector<leg_position>& leg_positions)
{
    ifstream infile;
    infile.open(file.data());   //将文件流对象与文件连接起来
    assert(infile.is_open());   //若失败,则输出错误消息,并终止程序运行
    string s;
    vector<float> res;
    leg_position leg;
    while(getline(infile,s))
    {
        if(s[0] == '2'){
            res.clear();

            while (!s.empty())
            {
                if (s.find(" ") == string::npos)
                {
                    res.push_back(stof(s));
                    s.clear();
                    break;
                }
          string s_temp = s.substr(0, s.find(" "));
          res.push_back(stof(s_temp));
          s.erase(0, s.find(" ") + 1);
            }
            leg.center = Point2f(res[1]*width,res[2]*height);
            leg.width = res[3]*width*1.5;
            leg.height = res[4]*height*1.5;
            float up_left_x = max(1.0f,leg.center.x -  leg.width/2);
            float up_left_y = max(1.0f,leg.center.y -  leg.height/2);
            float down_right_x = min((leg.center.x +  leg.width/2),width);
            float down_right_y = min(leg.center.y +  leg.height/2,height);
            leg.center = Point2f((up_left_x+down_right_x)/2,(up_left_y+down_right_y)/2);
            leg.width = -up_left_x+down_right_x;
            leg.height = -up_left_y+down_right_y;

            leg_positions.push_back(leg);
        }
    }
    infile.close();             //关闭文件输入流
}


void handlephoto (string p){
    string file_path;
    cv::Mat image ;
    vector<leg_position> leg_positions;
    leg_positions.clear();
    file_path=p + (".jpg");
    image = imread(file_path);
    readTxt(p+".txt",leg_positions);
    for (leg_position l: leg_positions){
     handletable(l, image);
     waitKey(0);
    if(l.height/l.width>0){
//    cv::rectangle(image,CvPoint(l.center.x+l.width/2,l.center.y+l.height/2),
//                  CvPoint(l.center.x-l.width/2,l.center.y-l.height/2),Scalar(0,255,255),2);
//    cv::line(image,CvPoint(l.center.x+l.width/2,l.center.y+l.height/2),
//                  CvPoint(l.center.x-l.width/2,l.center.y-l.height/2),Scalar(0,255,255),2);
//    cv::line(image,CvPoint(l.center.x+l.width/2,l.center.y-l.height/2),
//                  CvPoint(l.center.x-l.width/2,l.center.y+l.height/2),Scalar(0,255,255),2);
     }
    }
    imshow("原图",image);
}


int main(void)
{
    string path = "/home/oym/boocax_vo/pose_Of_TableLeg/photo_Of_tableLeg";
    vector<string> files=getFiles(path);
    for (string p:files){
    handlephoto("/home/oym/boocax_vo/pose_Of_TableLeg/photo_Of_tableLeg/"+p);
    waitKey(0);
    }

    return 0;
}





#endif
