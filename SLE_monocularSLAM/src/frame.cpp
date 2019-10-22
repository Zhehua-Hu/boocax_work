#include "../include/frame.h"

namespace SLE_monocularSLAM
{
Frame::Frame()
: id_(-1), time_stamp_(-1), camera_(nullptr), is_key_frame_(false),sle_frame(new cv::Sift_Line_Endpoint(0,3,0.04,10,1.6))
{

}

Frame::Frame ( long id, const Mat& gray,Camera::Ptr camera,double time_stamp, SE3 T_c_w,  Mat color, Mat depth )
: id_(id), gray_(gray),time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth), is_key_frame_(false)
,sle_frame(new cv::Sift_Line_Endpoint(0,3,0.04,10,1.6))
{
    sle_frame->detect_keypoints( gray_, keypoints_ , descriptors_ );
}

Frame::Frame ( long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera, Mat color, Mat depth )
: id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth), is_key_frame_(false)
{

}

Frame::~Frame()
{

}

Frame::Ptr Frame::createFrame(const Mat& gray, Camera::Ptr camera )
{
    static long factory_id = 0;
    return Frame::Ptr( new Frame(factory_id++, gray,camera));
}

Frame::Ptr Frame::createFrame()
{
    static long factory_id = 0;
    return Frame::Ptr( new Frame(factory_id++));
}

double Frame::findDepth ( const cv::KeyPoint& kp )
{
    int x = cvRound(kp.pt.x);
    int y = cvRound(kp.pt.y);
    ushort d = depth_.ptr<ushort>(y)[x];
    if ( d!=0 )
    {
        return double(d)/camera_->depth_scale_;
    }
    else 
    {
        // check the nearby points 
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        for ( int i=0; i<4; i++ )
        {
            d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];
            if ( d!=0 )
            {
                return double(d)/camera_->depth_scale_;
            }
        }
    }
    return -1.0;
}

void Frame::setPose ( const SE3& T_c_w )
{
    T_c_w_ = T_c_w;
}


Vector3d Frame::getCamCenter() const
{
    return T_c_w_.inverse().translation();
}

bool Frame::isInFrame ( const Vector3d& pt_world )
{
    Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );
    // cout<<"P_cam = "<<p_cam.transpose()<<endl;
    if ( p_cam(2,0)<0 ) return false;
    Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ );
    // cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
    return pixel(0,0)>0 && pixel(1,0)>0 
        && pixel(0,0)<color_.cols 
        && pixel(1,0)<color_.rows;
}


void Frame::UndistortKeyPoints(){

    int N = keypoints_.size();

    cv::Mat mK=camera_->mK.clone();

    mDistCoef=camera_->mDistCoef.clone();

    if(mDistCoef.at<float>(0)==0.0)
    {
     Unkeypoints = keypoints_;
        return;
    }

    // Fill matrix with points
    cv::Mat mat;//(N,2,CV_32F);
    mat = Mat::zeros(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=keypoints_[i].pt.x;
        mat.at<float>(i,1)=keypoints_[i].pt.y;
    }

#if Debug
    cout<<mat<<endl;
#endif
    // Undistort points
    mat=mat.reshape(2);
#if Debug
cout<<"mat.reshape(2) : "<<mat<<endl;
cout<<mDistCoef<<endl;
#endif

    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
#if Debug
    cout<<"After undistorted:"<<endl<<mat<<endl;
#endif

    mat=mat.reshape(1);
#if Debug
    cout<<"After undistorted,mat.reshape(1):"<<endl<<mat<<endl;
#endif

    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = keypoints_[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        Unkeypoints.push_back(kp);
    }
}


}
