#include"../include/common.h"
#if TEST_READFILE
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

struct leg_position{
    Point2f center;
    float width;
    float height;
};


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


int main(){
        DIR *dir;
        char basePath[100];
        string path = "/home/oym/boocax_vo/pose_Of_TableLeg/photo_Of_tableLeg";
        string file_path;
        vector<string> files=getFiles(path);
        vector<leg_position> leg_positions;
        cv::Mat image ;
        for (string p:files){
            leg_positions.clear();
            file_path=path+("/") + p + (".jpg");
            image = imread(file_path);
        readTxt(path+'/'+p+".txt",leg_positions);
        for (leg_position l: leg_positions){
            if(l.height/l.width>0){
            cv::rectangle(image,CvPoint(l.center.x+l.width/2,l.center.y+l.height/2),
                          CvPoint(l.center.x-l.width/2,l.center.y-l.height/2),Scalar(0,255,255),2);
            cv::line(image,CvPoint(l.center.x+l.width/2,l.center.y+l.height/2),
                          CvPoint(l.center.x-l.width/2,l.center.y-l.height/2),Scalar(0,255,255),2);
            cv::line(image,CvPoint(l.center.x+l.width/2,l.center.y-l.height/2),
                          CvPoint(l.center.x-l.width/2,l.center.y+l.height/2),Scalar(0,255,255),2);

             }
            }
        imshow(p,image);
        waitKey(0);

        }
        return 0;

}

#endif
