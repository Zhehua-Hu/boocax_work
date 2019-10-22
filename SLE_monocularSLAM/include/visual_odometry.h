#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H
#include "common.h"
#include "map.h"
#include "frame.h"
#include "sift_line_endpoint.h"
#include "camera.h"
#include "config.h"
#include "initialize_mono.h"

namespace SLE_monocularSLAM{

class VisualOdometry{
public:
    typedef shared_ptr<VisualOdometry> Ptr;
    enum VOstate {
        INITIALIZING = -1,
        OK = 0,
        LOST = 1,
        NOT_INITIALIZED = 2
    };
    bool first_initialize = 1;
    VOstate state_;
    Map::Ptr map_;
    Frame::Ptr ref_;
    Frame::Ptr curr_;
    Camera::Ptr                    camera_;     // Pinhole RGBD Camera model
   // Camera camera();    //相机初始化
   // cv::Ptr<cv::Sift_Line_Endpoint> sle_;
    std::vector<Frame::Ptr>  Keyframes;
    cv::Sift_Line_Endpoint::Ptr_SLE sle_;
    cv::Ptr<cv::ORB> orb_;
    vector<cv::KeyPoint> keypoints_ref_;    //self_add
    vector<cv::KeyPoint> keypoints_curr_;
    cv::Mat mDistCoef;
    cv::Mat mK;
    vector<cv::KeyPoint>  kpref_Un;
    vector<cv::KeyPoint>  kpcurr_Un;

    cv::Mat track = Mat::ones(1000,1000,CV_8UC1)*255;

    Mat descriptors_curr_;
    Mat descriptors_ref_;
    vector<DMatch> feature_matches_;
    SE3 T_c_w_estimated_;
    int num_inliers_;
    int num_lost_;
    int num_of_features_;
    int level_pyramid_;
    double scale_factor_;
    float match_ratio_=0.3;
    int max_num_lost_;
    int min_inliers_;
    double key_frame_min_rot;
    double key_frame_min_trans;
    cv::FlannBasedMatcher   matcher_flann_;     // flann matcher
    double  map_point_erase_ratio_; // remove map point ratio
    vector<MapPoint::Ptr>   match_3dpts_;       // matched 3d points
    vector<int>             match_2dkp_index_;  // matched 2d pixels (index of kp_curr)
    VisualOdometry();
    ~VisualOdometry();
    bool addFrame(Frame::Ptr frame);

    bool addimage(const Mat& image);

    Mat saved_pose;

    Vector3d    saved_frame_pos_;       // Frame Position in world

    bool add_MonocularFrame(Frame::Ptr frame);

    bool draw_trajactory();

    Mat cur_R ;
    Mat cur_t ;


    void pose_estimation_2d2d ( std::vector<KeyPoint> keypoints_1,
                                std::vector<KeyPoint> keypoints_2,
                                std::vector< DMatch > matches,
                                Mat& R, Mat& t );
    vector<DMatch> matches;



    initialize_mono* mono_initializer;

    std::vector<cv::Point3f> mvIniP3D;

    void computer_Rt_ref(Mat& R, Mat&t, const Frame::Ptr frame);

    void     UndistortKeyPoints();


protected:
    void extractKeyPoints();
    void Monocular_extractKeyPoints();
    void computeDescriptors();
    void Monocular_computeDescriptors();
    void featureMatching();
    void Monocular_featureMatching();

    void Monocular_SLEMatching();

    void Monocular_extractKeyPoints_descriptors();
    void setRef3DPoints();
    void optimizeMap();
    void addKeyFrame();
    void addMapPoints();
    bool checkEstimatedPose();
    bool checkKeyFrame();
    double getViewAngle( Frame::Ptr frame, MapPoint::Ptr point );

};

}







#endif // VISUAL_ODOMETRY_H
