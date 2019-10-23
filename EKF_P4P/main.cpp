#include "common.h"
float barcode_width = 0.05,barcode_height = 0.02,observe_distance = 0.5;

int main(int argc, char *argv[])
{
   std::string parameterFile = "/home/oym/boocax_vo/EKF-P4P/para.yaml";
   std::vector<cv::Vec3f> landmarks;
   std::default_random_engine random(time(NULL));
   std::uniform_real_distribution<float> dis1(0, 30);      //x向
   std::uniform_real_distribution<float> dis2(-5, 5);      //Y向
   std::uniform_real_distribution<float> dis3(0.5, 1.5);   //Z向,摄像头高度范围
   OutFile.open("test_PNP.txt",ios::app);
   float y_start =0,y_end = 20,z_start = 0.5,z_end = 1.5;
   float y=y_start,z=z_start;
   for(int i=0;i<100000;i++){
       y = y+barcode_width * i;
       z = 0;
       if(y>y_end) break;
       for(int j=0;j<100000;j++){
           z=z+j*barcode_height;
           if(z>z_end) break;
           landmarks.push_back(cv::Vec3f(observe_distance,y,z));
       }
   }
   System localization(landmarks,parameterFile);
   while(true){
   localization.run(0,1,0);
   cv::waitKey(0);
   }
   cv::waitKey(0);
   OutFile.close();
   return 0;
}
