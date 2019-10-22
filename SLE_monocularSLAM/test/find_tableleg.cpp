#include"../include/common.h"
#if FIND_TABLELEG
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
using namespace cv::line_descriptor;
float width = 1280;
float height = 720;


bool merge_parallel_line(const KeyLine kl1, const KeyLine kl2 , KeyLine& merge_kl);
struct leg_position{
    Point2f center;
    float width;
    float height;
};

struct kp_onlines{
    KeyPoint kp;
    vector<KeyLine> keylines;
    vector<float> distance;
    bool isconner;

};
double line_threshold=5;
double point_threshold=5;

double point_distance(Point2f p1, Point2f p2){
    return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}

bool isperpendicular(KeyLine kl1, KeyLine kl2){
    Point2f p1_1(kl1.endPointX,kl1.endPointY);
    Point2f p1_2(kl1.startPointX,kl1.startPointY);
    Point2f p2_1(kl2.endPointX,kl2.endPointY);
    Point2f p2_2(kl2.startPointX,kl2.startPointY);
    float   length1 = point_distance(p1_1,p1_2);
    float   length2 = point_distance(p2_1,p2_2);
    float cosangle =((p1_1.x-p1_2.x)*(p2_1.x-p2_2.x)+(p1_1.y-p1_2.y)*(p2_1.y-p2_2.y))/(length1*length2);
    float angle= acos(cosangle)*180/3.1415926;
    if (abs(angle) < 100 && abs(angle)>80 ) return true;
    else
        return false;

}

void check_ifiscorner(vector<kp_onlines>& kpol ){
    for (int i =0; i< kpol.size();i++){
        kpol[i].isconner = 0;
        int size = kpol[i].keylines.size();
        if(size<2) break;
        else {
            for(int j=0;j<size;j++){
                for(int k=0;k<size;k++){
                    if (isperpendicular(kpol[i].keylines[j], kpol[i].keylines[k] )) kpol[i].isconner = 1;
                }
            }
        }
    }

}




bool remove_duplicated_keypoints( vector<KeyPoint>  kps, vector<KeyPoint>& dst ){
    bool couldbemerged = 0;
    int size = kps.size();
    for (int i = 0;i<size;i++){
        bool inotmerged = 1;
        for (int j = i+1;j<size;j++){
             if(point_distance(kps[i].pt, kps[j].pt)<point_threshold){
                 KeyPoint temp(Point2f((kps[i].pt.x+kps[j].pt.x)/2, (kps[i].pt.y+kps[j].pt.y)/2 ),0);
                 dst.push_back(temp);
                 couldbemerged = 1;
                 inotmerged=0;
                 kps.erase(kps.begin()+j);
                 size = kps.size();
             }
    }
        if(inotmerged)                   dst.push_back(kps[i]);

    }

    if(!couldbemerged) return true;
     else {
        kps.clear();
        kps.assign(dst.begin(),dst.end());
        dst.clear();
        return remove_duplicated_keypoints(kps,dst);
    }

}


bool merge_line(const KeyLine kl1, const KeyLine kl2 , KeyLine& merge_kl){

    if(point_distance(Point2f(kl1.endPointX,kl1.endPointY), Point2f(kl2.endPointX,kl2.endPointY))<point_threshold){
        float angle = atan((kl1.endPointY-kl2.endPointY)/(kl1.endPointX-kl2.endPointX));
        if(angle>= min(kl1.angle , kl2.angle) && angle<= max(kl1.angle , kl2.angle)){
        merge_kl.endPointX=kl1.startPointX;
        merge_kl.endPointY=kl1.startPointY;
        merge_kl.startPointX=kl2.startPointX;
        merge_kl.startPointY=kl2.startPointY;
        merge_kl.angle = atan((merge_kl.endPointY-merge_kl.startPointY)/(merge_kl.endPointX-merge_kl.startPointX))*180/3.1415;
        merge_kl.lineLength = point_distance(Point2f(merge_kl.endPointX,merge_kl.endPointY),Point2f(merge_kl.startPointX,merge_kl.startPointY));
        return true;
        }
    }
   else if(point_distance(Point2f(kl1.endPointX,kl1.endPointY), Point2f(kl2.startPointX,kl2.startPointY))<point_threshold){
        float angle = atan((kl1.endPointY-kl2.startPointY)/(kl1.endPointX-kl2.startPointX));
        if(angle>= min(kl1.angle , kl2.angle) && angle<= max(kl1.angle , kl2.angle)){
        merge_kl.endPointX=kl1.startPointX;
        merge_kl.endPointY=kl1.startPointY;
        merge_kl.startPointX=kl2.endPointX;
        merge_kl.startPointY=kl2.endPointY;
        merge_kl.angle = atan((merge_kl.endPointY-merge_kl.startPointY)/(merge_kl.endPointX-merge_kl.startPointX))*180/3.1415;
        merge_kl.lineLength = point_distance(Point2f(merge_kl.endPointX,merge_kl.endPointY),Point2f(merge_kl.startPointX,merge_kl.startPointY));
        return true;
        }
    }

    else if(point_distance(Point2f(kl1.startPointX,kl1.startPointY), Point2f(kl2.startPointX,kl2.startPointY))<point_threshold){
        float angle = atan((kl1.startPointY-kl2.startPointY)/(kl1.startPointX-kl2.startPointX));
        if(angle>= min(kl1.angle , kl2.angle) && angle<= max(kl1.angle , kl2.angle)){
         merge_kl.endPointX=kl1.endPointX;
         merge_kl.endPointY=kl1.endPointY;
         merge_kl.startPointX=kl2.endPointX;
         merge_kl.startPointY=kl2.endPointY;
         merge_kl.angle = atan((merge_kl.endPointY-merge_kl.startPointY)/(merge_kl.endPointX-merge_kl.startPointX))*180/3.1415;
         merge_kl.lineLength = point_distance(Point2f(merge_kl.endPointX,merge_kl.endPointY),Point2f(merge_kl.startPointX,merge_kl.startPointY));
         return true;
    }
     }

    else if(point_distance(Point2f(kl1.startPointX,kl1.startPointY), Point2f(kl2.endPointX,kl2.endPointY))<point_threshold){
        float angle = atan((kl1.startPointY-kl2.endPointY)/(kl1.startPointX-kl2.endPointX));
        if(angle>= min(kl1.angle , kl2.angle) && angle<= max(kl1.angle , kl2.angle)){
         merge_kl.endPointX=kl1.endPointX;
         merge_kl.endPointY=kl1.endPointY;
         merge_kl.startPointX=kl2.startPointX;
         merge_kl.startPointY=kl2.startPointY;
         merge_kl.angle = atan((merge_kl.endPointY-merge_kl.startPointY)/(merge_kl.endPointX-merge_kl.startPointX))*180/3.1415;
         merge_kl.lineLength = point_distance(Point2f(merge_kl.endPointX,merge_kl.endPointY),Point2f(merge_kl.startPointX,merge_kl.startPointY));
         return true;
        }
     }
    else
        return false;


}






bool mergin_segment(vector<KeyLine> origin, vector<KeyLine>& dst){
    bool couldbemerged = 0;
    int size = origin.size();
    for (int i=0; i<size;i++){
          bool inotmerged = 1;
        for(int j=i+1;j<size;j++){
            if (abs (origin[i].angle - origin[j].angle)<=line_threshold){
                KeyLine temp1;
                if(merge_line(origin[i],origin[j],temp1)){
                    dst.push_back(temp1);
                    couldbemerged = 1;
                    inotmerged=0;
                    origin.erase(origin.begin()+j);
                    size = origin.size();
                }
            }
        }
        if (inotmerged) dst.push_back(origin[i]);
    }
   if(!couldbemerged) return true;
    else {
       origin.clear();
       origin.assign(dst.begin(),dst.end());
       dst.clear();
       return mergin_segment(origin,dst);
   }

}


double PointToSegDist(Point2f p, KeyLine kl)   //点到线段的距离
{
double x=p.x, y=p.y, x1=kl.endPointX,  y1=kl.endPointY,   x2=kl.startPointX,   y2=kl.startPointY;
double cross = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);
if (cross <= 0) return sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));

double d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
if (cross >= d2) return sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2));

double r = cross / d2;
double px = x1 + (x2 - x1) * r;
double py = y1 + (y2 - y1) * r;
return sqrt((x - px) * (x - px) + (py - y) * (py - y));
}


float d_point_Line(Point2f p, KeyLine kl ){
    double x=p.x, y=p.y, x1=kl.endPointX,  y1=kl.endPointY,   x2=kl.startPointX,   y2=kl.startPointY;
    double cross = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);
    double d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
    double r = cross / d2;
    double px = x1 + (x2 - x1) * r;
    double py = y1 + (y2 - y1) * r;
    return sqrt((x - px) * (x - px) + (py - y) * (py - y));
}



bool merge_parallel_line(const KeyLine kl1, const KeyLine kl2 , KeyLine& merge_kl){
     if(d_point_Line(Point2f(kl1.endPointX,kl1.endPointY),kl2)<point_threshold ||
             d_point_Line(Point2f(kl1.startPointX,kl1.startPointY),kl2)<point_threshold){
      float d1 =  point_distance(Point2f(kl2.endPointX,kl2.endPointY),Point2f(kl1.endPointX,kl1.endPointY));
      float d2 =  point_distance(Point2f(kl2.endPointX,kl2.endPointY),Point2f(kl1.startPointX,kl1.startPointY));
      float d3 =  point_distance(Point2f(kl2.startPointX,kl2.startPointY),Point2f(kl1.endPointX,kl1.endPointY));
      float d4 =  point_distance(Point2f(kl2.startPointX,kl2.startPointY),Point2f(kl1.startPointX,kl1.startPointY));
      float max_d = max(d1,max(d2,max(d3,d4)));
      if(d1 == max_d){
          merge_kl.endPointX=kl1.endPointX;
          merge_kl.endPointY=kl1.endPointY;
          merge_kl.startPointX=kl2.endPointX;
          merge_kl.startPointY=kl2.endPointY;
          merge_kl.angle = atan((merge_kl.endPointY-merge_kl.startPointY)/(merge_kl.endPointX-merge_kl.startPointX))*180/3.1415;
          merge_kl.lineLength = point_distance(Point2f(merge_kl.endPointX,merge_kl.endPointY),Point2f(merge_kl.startPointX,merge_kl.startPointY));
          return true;
      }
     else if(d2 == max_d){
          merge_kl.endPointX=kl1.startPointX;
          merge_kl.endPointY=kl1.startPointY;
          merge_kl.startPointX=kl2.endPointX;
          merge_kl.startPointY=kl2.endPointY;
          merge_kl.angle = atan((merge_kl.endPointY-merge_kl.startPointY)/(merge_kl.endPointX-merge_kl.startPointX))*180/3.1415;
          merge_kl.lineLength = point_distance(Point2f(merge_kl.endPointX,merge_kl.endPointY),Point2f(merge_kl.startPointX,merge_kl.startPointY));
          return true;
      }
      if(d3 == max_d){
          merge_kl.endPointX=kl1.endPointX;
          merge_kl.endPointY=kl1.endPointY;
          merge_kl.startPointX=kl2.startPointX;
          merge_kl.startPointY=kl2.startPointY;
          merge_kl.angle = atan((merge_kl.endPointY-merge_kl.startPointY)/(merge_kl.endPointX-merge_kl.startPointX))*180/3.1415;
          merge_kl.lineLength = point_distance(Point2f(merge_kl.endPointX,merge_kl.endPointY),Point2f(merge_kl.startPointX,merge_kl.startPointY));
          return true;
      }
      if(d4 == max_d){
          merge_kl.endPointX=kl1.startPointX;
          merge_kl.endPointY=kl1.startPointY;
          merge_kl.startPointX=kl2.startPointX;
          merge_kl.startPointY=kl2.startPointY;
          merge_kl.angle = atan((merge_kl.endPointY-merge_kl.startPointY)/(merge_kl.endPointX-merge_kl.startPointX))*180/3.1415;
          merge_kl.lineLength = point_distance(Point2f(merge_kl.endPointX,merge_kl.endPointY),Point2f(merge_kl.startPointX,merge_kl.startPointY));
          return true;
      }

    }
    else
        return false;
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


void handletable(leg_position l, Mat img ){
    RNG rng(12345);
    cv::Mat image = img.clone() ;
    Mat ROI,ROI_1,ROI_2,ROI_3,ROI_4,ROI_5,ROI_6 ,ROI_7,ROI_8,ROI_9,ROI_10,ROI_11;
    double min_length=0;
    Point2f up_left(l.center.x-l.width/2,l.center.y-l.height/2);
    Point2f down_right(l.center.x+l.width/2,l.center.y+l.height/2);
    up_left.x = max(1.0f,up_left.x);
    up_left.y = max(1.0f,up_left.y);
    Rect rect(up_left.x,up_left.y, l.width, l.height);
    ROI_1 = image(rect);
    ROI_2 = ROI_1.clone();// image2(rect);
    ROI_4 = ROI_1.clone();
    ROI = ROI_1.clone();
    ROI_5 = ROI_1.clone();
    ROI_6 = ROI_1.clone();
    ROI_7 = ROI_1.clone();
    ROI_8 = ROI_1.clone();
    ROI_9 = ROI_1.clone();
    min_length = min(l.width/4, l.height/4);
    Ptr<LSDDetectorC> lsd = LSDDetectorC::createLSDDetectorC();
    LSDDetectorC::LSDOptions options;
    options.refine       = LSD_REFINE_ADV;
    options.scale        = 0.8;
    options.sigma_scale  = 0.6;
    options.quant        = 2.0;
    options.ang_th       = 22.5;
    options.log_eps      = 1.0;
    options.density_th   = 0.6;
    options.n_bins       = 1024;
    options.min_length   = 10;
    std::vector<KeyLine> keylines, selected_keylines,merged_keylines,s_merged_keylines;
    cvtColor(ROI_2,ROI_2,CV_BGR2GRAY);
    std::vector<Mat> ROI_Mat;
    ROI_Mat.push_back(ROI_2);
    lsd->detectline(ROI_Mat, keylines,options);
    int margin =5 , width=ROI_2.cols,  height = ROI_2.rows;
    std::for_each(keylines.begin(), keylines.end(), [&](KeyLine kl)
    {
    cv::Scalar color = cv::Scalar(255, 255, 255);
    int scale = 1;
    cv::line(ROI_1,cv::Point(kl.startPointX*scale,kl.startPointY*scale),cv::Point(kl.endPointX*scale,kl.endPointY*scale),color,1.5);
    cv::line(ROI_4,cv::Point(kl.startPointX*scale,kl.startPointY*scale),cv::Point(kl.endPointX*scale,kl.endPointY*scale),color,1.5);
    cv::circle(ROI_4,cv::Point(kl.endPointX,kl.endPointY),1.5,cv::Scalar(255,255,255),2);
    cv::circle(ROI_4,cv::Point(kl.startPointX,kl.startPointY),1.5,cv::Scalar(255,255,255),2);
     // 去掉端点在图像边缘的线段
    if (kl.startPointX>margin && kl.startPointX<width-margin &&
        kl.endPointX>margin && kl.endPointX<width-margin     &&
        kl.startPointY>margin && kl.startPointY<height-margin &&
        kl.endPointY>margin && kl.endPointY<height-margin){
        selected_keylines.push_back(kl);
        cv::circle(ROI_2,cv::Point(kl.endPointX,kl.endPointY),1.5,cv::Scalar(255,255,255),2);
        cv::circle(ROI_2,cv::Point(kl.startPointX,kl.startPointY),1.5,cv::Scalar(255,255,255),2);
        cv::line(ROI_2,cv::Point(kl.startPointX*scale,kl.startPointY*scale),cv::Point(kl.endPointX*scale,kl.endPointY*scale),color,1.5);
    }
    });

    {
     mergin_segment(selected_keylines,merged_keylines);
    }
    vector<KeyPoint> merged_kps, s_merged_kps;
    for(KeyLine kl : merged_keylines){
        if(kl.lineLength >min_length){
        s_merged_keylines.push_back(kl);
        cv::line(ROI_6,cv::Point(kl.startPointX,kl.startPointY),cv::Point(kl.endPointX,kl.endPointY),Scalar(255, 255, 255),1.5);
        cv::circle(ROI_6,cv::Point(kl.endPointX,kl.endPointY),1.5,cv::Scalar(255,255,255),2);
        cv::circle(ROI_6,cv::Point(kl.startPointX,kl.startPointY),1.5,cv::Scalar(255,255,255),2);
        cv::line(ROI_8,cv::Point(kl.startPointX,kl.startPointY),cv::Point(kl.endPointX,kl.endPointY),Scalar(255, 255, 255),1.5);
        KeyPoint kp;
        kp.pt.x=kl.startPointX;
        kp.pt.y=kl.startPointY;
        merged_kps.push_back(kp);
        kp.pt.x=kl.endPointX;
        kp.pt.y=kl.endPointY;
        merged_kps.push_back(kp);
        }
    }
    ROI_9 = ROI_8.clone();
    remove_duplicated_keypoints(merged_kps,s_merged_kps);

    for(KeyPoint kp : s_merged_kps){

        cv::circle(ROI_9,cv::Point(kp.pt.x,kp.pt.y),1.5,cv::Scalar(255,255,255),2);
    }

    for(KeyLine kl : selected_keylines){
        if(kl.lineLength > min_length){
        cv::line(ROI_7,cv::Point(kl.startPointX,kl.startPointY),cv::Point(kl.endPointX,kl.endPointY),Scalar(255, 255, 255),1.5);
                                       }
    }

    ROI_3 = ROI_1.clone();
    vector<KeyPoint> keypoints;
    for (KeyLine kl: keylines){
        KeyPoint kp;
        kp.pt.x=kl.startPointX;
        kp.pt.y=kl.startPointY;
        keypoints.push_back(kp);
        kp.pt.x=kl.endPointX;
        kp.pt.y=kl.endPointY;
        keypoints.push_back(kp);
    }

    ROI_11=ROI_1.clone();
    for(KeyPoint kp : keypoints){

        cv::circle(ROI_11,cv::Point(kp.pt.x,kp.pt.y),1.5,cv::Scalar(255,255,255),2);
    }

    vector<KeyPoint> kps;

    remove_duplicated_keypoints(keypoints,kps);


    for(KeyPoint kp : kps){

        cv::circle(ROI_3,cv::Point(kp.pt.x,kp.pt.y),1.5,cv::Scalar(255,255,255),2);
    }


    vector<kp_onlines>   KPonlines;
    for (KeyPoint kp:kps ){
        int i =0;
        float dis = 0;
        kp_onlines kponlines;
        kponlines.kp = kp;
        for(i=0;i<s_merged_keylines.size();i++)
        {
            dis = d_point_Line(Point2f(kp.pt.x,kp.pt.y), s_merged_keylines[i] );
            if(dis <3){
            kponlines.keylines.push_back(s_merged_keylines[i]);
            kponlines.distance.push_back(dis);
            }
        }
        if(kponlines.keylines.size()>1) KPonlines.push_back(kponlines);
    }
ROI_5 = ROI_8.clone();
ROI_10 = ROI_8.clone();

for (kp_onlines kpol : KPonlines){
    cv::circle(ROI_5,cv::Point(kpol.kp.pt.x,kpol.kp.pt.y),1.5,cv::Scalar(255,255,255),2);
}


check_ifiscorner(KPonlines);
for (kp_onlines kpol : KPonlines){
    if(kpol.isconner){
    cv::circle(ROI_10,cv::Point(kpol.kp.pt.x,kpol.kp.pt.y),1.5,cv::Scalar(255,255,255),2);
    }
}
//    imshow("0",image);
    namedWindow("1", CV_WINDOW_NORMAL);
    namedWindow("2", CV_WINDOW_NORMAL);
    namedWindow("3", CV_WINDOW_NORMAL);
    namedWindow("4", CV_WINDOW_NORMAL);
    namedWindow("5", CV_WINDOW_NORMAL);
    namedWindow("6", CV_WINDOW_NORMAL);
    namedWindow("7", CV_WINDOW_NORMAL);
//    namedWindow("8", CV_WINDOW_NORMAL);
    namedWindow("9", CV_WINDOW_NORMAL);
    namedWindow("10", CV_WINDOW_NORMAL);
    namedWindow("11", CV_WINDOW_NORMAL);

    moveWindow("1",ROI_1.cols*4,50);
    moveWindow("2",ROI_1.cols*7,50);
    moveWindow("3",ROI_1.cols*10,50);
    moveWindow("4",ROI_1.cols*13,50);
    moveWindow("5",ROI_1.cols*16,50);

    moveWindow("6",ROI_1.cols*4,550);
    moveWindow("7",ROI_1.cols*7,550);
 //   moveWindow("8",ROI_1.cols*7,550);
    moveWindow("9",ROI_1.cols*10,550);
    moveWindow("10",ROI_1.cols*13,550);
    moveWindow("11",ROI_1.cols*16,550);

    cvResizeWindow("1", ROI_1.cols*2, ROI_1.rows+100); //创建一个固定值大小的窗口
    cvResizeWindow("2", ROI_1.cols*2, ROI_1.rows+100); //创建一个固定值大小的窗口
    cvResizeWindow("3", ROI_1.cols*2, ROI_1.rows+100); //创建一个固定值大小的窗口
    cvResizeWindow("4", ROI_1.cols*2, ROI_1.rows+100); //创建一个固定值大小的窗口
    cvResizeWindow("6", ROI_1.cols*2, ROI_1.rows+100); //创建一个固定值大小的窗口
    cvResizeWindow("5", ROI_1.cols*2, ROI_1.rows+100); //创建一个固定值大小的窗口
    cvResizeWindow("7", ROI_1.cols*2, ROI_1.rows+100); //创建一个固定值大小的窗口
//    cvResizeWindow("8", ROI_1.cols*2, ROI_1.rows+100); //创建一个固定值大小的窗口
    cvResizeWindow("9", ROI_1.cols*2, ROI_1.rows+100); //创建一个固定值大小的窗口
    cvResizeWindow("10", ROI_1.cols*2, ROI_1.rows+100); //创建一个固定值大小的窗口
    cvResizeWindow("11", ROI_1.cols*2, ROI_1.rows+100); //创建一个固定值大小的窗口
    imshow("1",ROI_1);   //显示所有的线
    imshow("2",ROI_2);   //显示去掉端点在边缘的线
    imshow("3",ROI_3);   //合并得很近的端点
    imshow("4",ROI_4);   //显示所有的线段和端点
    imshow("5",ROI_5);   //合并线+合并点+过滤非交点
    imshow("6",ROI_6);   //合并的线+点+去掉端点在边缘的线
    imshow("7",ROI_7);   //筛选长度+去掉边缘线
//    imshow("8",ROI_8);   //合并线+去掉端点在边缘的线
    imshow("9",ROI_9);   //合并线+合并点
    imshow("10",ROI_10);   //合并线+合并点+过滤非交点+过滤没有相互垂直线的点
    imshow("11",ROI_11);   //所有端点

}

void handlephoto (string p){
    string file_path;
    cv::Mat image ;
    vector<leg_position> leg_positions;
    leg_positions.clear();
    file_path=p + (".jpg");
    image = imread(file_path);
    Mat temp_image=image.clone();
    readTxt(p+".txt",leg_positions);
    for (leg_position l: leg_positions){
     handletable(l, image);
     while(1){
      char key = waitKey(0);
      if(key == ' ')  break;
     }    if(l.height/l.width>0){
//#if Debug
    cv::rectangle(temp_image,CvPoint(l.center.x+l.width/2,l.center.y+l.height/2),
                  CvPoint(l.center.x-l.width/2,l.center.y-l.height/2),Scalar(0,255,255),2);
    cv::line(temp_image,CvPoint(l.center.x+l.width/2,l.center.y+l.height/2),
                  CvPoint(l.center.x-l.width/2,l.center.y-l.height/2),Scalar(0,255,255),2);
    cv::line(temp_image,CvPoint(l.center.x+l.width/2,l.center.y-l.height/2),
                  CvPoint(l.center.x-l.width/2,l.center.y+l.height/2),Scalar(0,255,255),2);
//#endif
     }
    }
    imshow("原图",temp_image );

}


int main(void)
{
    string path = "/home/oym/boocax_vo/pose_Of_TableLeg/photo_Of_tableLeg";
    vector<string> files=getFiles(path);
    for (string p:files){
    handlephoto("/home/oym/boocax_vo/pose_Of_TableLeg/photo_Of_tableLeg/"+p);
    while(1){
     char key = waitKey(0);
     if(key == ' ')  break;
    }
    }
    return 0;
}

#endif
