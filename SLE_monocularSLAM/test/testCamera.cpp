#include "include/visual_odometry.h"

#if TEST_CAMERA


int main(int argc, char *argv[])
{
    string file_dir = "default.yaml";
    SLE_monocularSLAM::Config::setParameterFile ( file_dir);
    SLE_monocularSLAM::VisualOdometry::Ptr vo ( new SLE_monocularSLAM::VisualOdometry );
    SLE_monocularSLAM::Camera::Ptr camera ( new SLE_monocularSLAM::Camera );
    SLE_monocularSLAM::Frame::Ptr pFrame = SLE_monocularSLAM::Frame::createFrame();
    pFrame->camera_ = camera;
    cv::VideoCapture cap(1);
    while(1){
    cv::Mat frame;
    cap >> frame;   // 读取相机数据
    imshow("image",frame);
    if ( frame.data == nullptr ) break;

    cvtColor(frame,frame,CV_BGR2GRAY);
    pFrame->gray_ = frame;

    vo->addFrame ( pFrame );

    char key = (char)waitKey(50);

     if (key == 'q') break;

    }

    return 0;
}

#endif
