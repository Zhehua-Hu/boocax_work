#include "visual_odometry.h"

#if TEST_VEDIO

int main(int argc, char *argv[])
{
    string outputVideoPath = "../vedio/test_color.mp4";  //视频文件
    string file_dir = "../config/default.yaml";   //配置文件
    SLE_monocularSLAM::Config::setParameterFile ( file_dir);
    SLE_monocularSLAM::VisualOdometry::Ptr vo ( new SLE_monocularSLAM::VisualOdometry );
    SLE_monocularSLAM::Camera::Ptr camera ( new SLE_monocularSLAM::Camera );
    vo->camera_=camera;
    cv::VideoCapture cap;
    cap.open(outputVideoPath);
    if(!cap.isOpened())
   {

    cout<<"can not open ...\n"<<endl;

   }

    int i=0;

    cv::Mat frame;

    while(1){

    i++;

    cap >> frame;

    if (frame.empty()) break;

    cvtColor(frame,frame,CV_BGR2GRAY);

    equalizeHist(frame,frame);

    imshow("src_image",frame);

    Mat dst = frame.clone();

    imshow("Equal_Hist",frame);

    vo->addimage ( frame );

    char key = (char)waitKey(50);

     if (key == 'q') break;

    }

    cout<<"总帧数目为:"<<i<<endl;


    cout<<"关键帧数目为:"<<vo->Keyframes.size()<<endl;

   waitKey(0);
    return 0;
}


#endif
