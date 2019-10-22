/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "../include/visual_odometry.h"
#include "../include/g2o_types.h"




namespace SLE_monocularSLAM
{

struct compare_match_by_distance
{
    inline bool operator()(const DMatch& match1, const DMatch& match2){
        return (match1.distance < match2.distance);
    }
};

VisualOdometry::VisualOdometry() :
    state_ ( NOT_INITIALIZED ), sle_(new cv::Sift_Line_Endpoint(0,3,0.04,10,1.6)),mono_initializer(static_cast<initialize_mono*>(NULL)),
    ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 ), matcher_flann_ ( new cv::flann::LshIndexParams ( 5,10,2 ) )
{
    num_of_features_    = Config::get<int> ( "number_of_features" );
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    match_ratio_        = Config::get<float> ( "match_ratio" );
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
    map_point_erase_ratio_ = Config::get<double> ( "map_point_erase_ratio" );
    orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
}

VisualOdometry::~VisualOdometry()
{

}

Mat temp;
bool VisualOdometry::addFrame ( Frame::Ptr frame )
{
    switch ( state_ )
    {
    case NOT_INITIALIZED:
    {

#if Debug

        cout<<"---------------------"<<endl;

        cout<<"state: NOT_INITIALIZED "<<endl;

#endif

        cv::imshow("frame1", frame->gray_);

        frame->frame_pos_<<0,0,0;

        curr_ = ref_ = frame;      

        temp=frame->gray_;

        Monocular_extractKeyPoints_descriptors();

        if(keypoints_curr_.size() < 50) break;

        descriptors_ref_=descriptors_curr_.clone();

        keypoints_ref_.assign(keypoints_curr_.begin(),keypoints_curr_.end());

        state_ = INITIALIZING;

        break;
    }

    case OK:
    {      

        Mat R,t;

        computer_Rt_ref(R, t, frame);

        temp = frame->gray_;



#if Debug

        cout<<"++++++++++++++++++"<<endl;

        cout<<"state: OK "<<endl;
#endif

        break;

    }

    case INITIALIZING:
    {
  //      cv::imshow("curr_image1", curr_->gray_);
  //      cv::imshow("curr_ref1", ref_->gray_);
#if Debug

        cout<<"+-+-+-+-+-+-+-+-+-+-+-+-+-+"<<endl;

        cout<<"state: INITIALIZING "<<endl;

#endif

        curr_ = frame;

        curr_->T_c_w_ = ref_->T_c_w_;

        keypoints_curr_.clear();

        Monocular_extractKeyPoints_descriptors();

        if(keypoints_curr_.size() < 50) break;

        Monocular_featureMatching();

        if (feature_matches_.size()<50) {

            state_ = NOT_INITIALIZED;

            break;
        }

        cv:Mat K=frame->camera_->mK;

        UndistortKeyPoints();

     //   mono_initializer = new initialize_mono(keypoints_ref_,keypoints_curr_,feature_matches_ , 1,100, K);

        mono_initializer = new initialize_mono(kpref_Un,kpcurr_Un,feature_matches_ , 1,100, K);

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        mono_initializer->Initialize(Rcw,tcw,mvIniP3D,vbTriangulated);

        cout<<"Rcw"<<Rcw<<endl;

        cout<<"-----------"<<endl;

        cout<<"tcw"<<tcw<<endl;

        delete mono_initializer;

        if(Rcw.data != NULL){

        cv::Mat pos02 = -Rcw.t()*tcw;  //第二副图原点在世界坐标系中的位置

//      pos02.convertTo(pos02, CV_32FC1);

        curr_->frame_pos_<< pos02.at<float>(0,0),pos02.at<float>(0,1),pos02.at<float>(0,2);

        state_ = OK;

        Keyframes.push_back(ref_);

        Keyframes.push_back(curr_);

        }

        else      state_ = NOT_INITIALIZED;


        break;

    }
    case LOST:
    {
        cout<<"vo has lost."<<endl;
        break;
    }
    }

    return true;
}



void VisualOdometry::extractKeyPoints()
{
    boost::timer timer;
    orb_->detect ( curr_->color_, keypoints_curr_ );
    cout<<"extract keypoints cost time: "<<timer.elapsed() <<endl;
}

void VisualOdometry::computeDescriptors()
{
    boost::timer timer;
    orb_->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );
    cout<<"descriptor computation cost time: "<<timer.elapsed() <<endl;
}

void VisualOdometry::featureMatching()
{
    boost::timer timer;
    vector<cv::DMatch> matches;
    // select the candidates in map 
    Mat desp_map;
    vector<MapPoint::Ptr> candidate;
    for ( auto& allpoints: map_->map_points_ )
    {
        MapPoint::Ptr& p = allpoints.second;
        // check if p in curr frame image 
        if ( curr_->isInFrame(p->pos_) )
        {
            // add to candidate 
            p->visible_times_++;
            candidate.push_back( p );
            desp_map.push_back( p->descriptor_ );
        }
    }
    
    matcher_flann_.match ( desp_map, descriptors_curr_, matches );
    // select the best matches
    float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;

    match_3dpts_.clear();
    match_2dkp_index_.clear();
    for ( cv::DMatch& m : matches )
    {
        if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            match_3dpts_.push_back( candidate[m.queryIdx] );
            match_2dkp_index_.push_back( m.trainIdx );
        }
    }
    cout<<"good matches: "<<match_3dpts_.size() <<endl;
    cout<<"match cost time: "<<timer.elapsed() <<endl;
}



bool VisualOdometry::checkEstimatedPose()
{
    // check if the estimated pose is good
    if ( num_inliers_ < min_inliers_ )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();
    if ( d.norm() > 5.0 )
    {
        cout<<"reject because motion is too large: "<<d.norm() <<endl;
        return false;
    }
    return true;
}

bool VisualOdometry::checkKeyFrame()
{
    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
        return true;
    return false;
}

void VisualOdometry::addKeyFrame()
{
    if ( map_->keyframes_.empty() )
    {
        // first key-frame, add all 3d points into map
        for ( size_t i=0; i<keypoints_curr_.size(); i++ )
        {
            double d = curr_->findDepth ( keypoints_curr_[i] );
            if ( d < 0 ) 
                continue;
            Vector3d p_world = ref_->camera_->pixel2world (
                Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), curr_->T_c_w_, d
            );
            Vector3d n = p_world - ref_->getCamCenter();
            n.normalize();
            MapPoint::Ptr map_point = MapPoint::createMapPoint(
                p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
            );
            map_->insertMapPoint( map_point );
        }
    }
    
    map_->insertKeyFrame ( curr_ );
    ref_ = curr_;
}

void VisualOdometry::addMapPoints()
{
    // add the new map points into map
    vector<bool> matched(keypoints_curr_.size(), false); 
    for ( int index:match_2dkp_index_ )
        matched[index] = true;
    for ( int i=0; i<keypoints_curr_.size(); i++ )
    {
        if ( matched[i] == true )   
            continue;
        double d = ref_->findDepth ( keypoints_curr_[i] );
        if ( d<0 )  
            continue;
        Vector3d p_world = ref_->camera_->pixel2world (
            Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), 
            curr_->T_c_w_, d
        );
        Vector3d n = p_world - ref_->getCamCenter();
        n.normalize();
        MapPoint::Ptr map_point = MapPoint::createMapPoint(
            p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
        );
        map_->insertMapPoint( map_point );
    }
}

void VisualOdometry::optimizeMap()
{
    // remove the hardly seen and no visible points 
    for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end(); )
    {
        if ( !curr_->isInFrame(iter->second->pos_) )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
        if ( match_ratio < map_point_erase_ratio_ )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        
        double angle = getViewAngle( curr_, iter->second );
        if ( angle > M_PI/6. )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        if ( iter->second->good_ == false )
        {
            // TODO try triangulate this map point 
        }
        iter++;
    }
    
    if ( match_2dkp_index_.size()<100 )
        addMapPoints();
    if ( map_->map_points_.size() > 1000 )  
    {
        // TODO map is too large, remove some one 
        map_point_erase_ratio_ += 0.05;
    }
    else 
        map_point_erase_ratio_ = 0.1;
    cout<<"map points: "<<map_->map_points_.size()<<endl;
}

double VisualOdometry::getViewAngle ( Frame::Ptr frame, MapPoint::Ptr point )
{
    Vector3d n = point->pos_ - frame->getCamCenter();
    n.normalize();
    return acos( n.transpose()*point->norm_ );
}

void VisualOdometry::Monocular_extractKeyPoints_descriptors(){
    boost::timer timer;

    sle_->detect_keypoints( curr_->gray_, keypoints_curr_ , descriptors_curr_ );

    cout<<"extract keypoints cost time: "<<timer.elapsed() <<endl;
}


void VisualOdometry::Monocular_featureMatching(){

    BFMatcher matcher;

    matches.clear();

    matcher.match(descriptors_ref_,descriptors_curr_ , matches);

    sort(matches.begin(),matches.end() ,compare_match_by_distance());

    feature_matches_.assign(matches.begin(),matches.begin()+std::min((int)(matches.size()*match_ratio_),100));

#if Debug

    Mat imageOutput;

    drawMatches(temp, keypoints_ref_,curr_->gray_, keypoints_curr_,  feature_matches_, imageOutput);

    cout<<"匹配的点数为"<<feature_matches_.size()<<endl;

    cv::putText(imageOutput,to_string(feature_matches_.size()),Point(640,50),FONT_HERSHEY_SIMPLEX,1,Scalar(255,23,0),4,8);

    cv::imshow("前后两帧匹配图", imageOutput);

//    cv::imshow("curr_image", curr_->gray_);

//    cv::imshow("curr_ref", ref_->gray_);

//    cv::imshow("temp:",temp);

#endif

}


void VisualOdometry:: Monocular_SLEMatching(){

    BFMatcher matcher( NORM_L2,true);

    matches.clear();

    matcher.match(ref_->descriptors_,curr_->descriptors_ , matches);

    sort(matches.begin(),matches.end() ,compare_match_by_distance());

    feature_matches_.assign(matches.begin(),matches.begin()+std::min((int)(matches.size()*match_ratio_),100));

#if ShowMatch

    Mat imageOutput;

    drawMatches(ref_->gray_, ref_->keypoints_,curr_->gray_, curr_->keypoints_,  feature_matches_, imageOutput);

    cout<<"匹配的点数为"<<feature_matches_.size()<<endl;

    cv::putText(imageOutput,to_string(feature_matches_.size()),Point(640,50),FONT_HERSHEY_SIMPLEX,1,Scalar(255,23,0),4,8);

    cv::namedWindow("前后两帧匹配图", WINDOW_NORMAL);

    Mat temp_img;

    resize(imageOutput,temp_img,Size(ref_->gray_.cols,ref_->gray_.rows),0, 0, INTER_LINEAR );

    cv::imshow("前后两帧匹配图", temp_img);

#endif


}

void VisualOdometry::pose_estimation_2d2d(std::vector<KeyPoint> keypoints_1, std::vector<KeyPoint> keypoints_2, std::vector<DMatch> matches, Mat &R, Mat &t){
    // 相机内参,TUM Freiburg2


    Mat K = ( Mat_<double> ( 3,3 ) << curr_->camera_->fx_, 0, curr_->camera_->cx_, 0, curr_->camera_->fy_, curr_->camera_->cy_, 0, 0, 1 );

    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }

    //-- 计算基础矩阵
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat ( points1, points2, FM_RANSAC );
    cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    //-- 计算本质矩阵
    Point2d principal_point ( 325.1, 249.7 );	//相机光心, TUM dataset标定值
    double focal_length = 521;			//相机焦距, TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    //-- 计算单应矩阵
    Mat homography_matrix;
    homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
    cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    cout<<"R is "<<endl<<R<<endl;
    cout<<"t is "<<endl<<t<<endl;
}


void VisualOdometry::computer_Rt_ref(Mat& R, Mat&t, const Frame::Ptr frame){

//    Frame::Ptr save_ref;

//    save_ref=ref;

//    Mat save_descriptors_ref = descriptors_ref_.clone();

//    vector<cv::KeyPoint> save_kps_ref;

//    save_kps_ref.assign(keypoints_ref_.begin(),keypoints_ref_.end());

    ref_=curr_;

    descriptors_ref_=descriptors_curr_.clone();

    keypoints_ref_.clear();

    keypoints_ref_.assign(keypoints_curr_.begin(),keypoints_curr_.end());

    curr_ = frame;

    keypoints_curr_.clear();

    feature_matches_.clear();

    Monocular_extractKeyPoints_descriptors();

    Monocular_featureMatching();

    if (feature_matches_.size()<50) {

//        ref_=save_ref;

//        descriptors_ref_=save_descriptors_ref.clone();

//        keypoints_ref_.clear();

//        keypoints_ref_.assign(save_kps_ref.begin(),save_kps_ref.end());

        return;
    }


    cv:Mat K=frame->camera_->mK;

    mono_initializer = new initialize_mono(keypoints_ref_,keypoints_curr_,feature_matches_ , 1,100, K);

    cv::Mat Rcw; // Current Camera Rotation
    cv::Mat tcw; // Current Camera Translation
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

    mono_initializer->Initialize(Rcw,tcw,mvIniP3D,vbTriangulated,1);

    cout<<"Rcw"<<Rcw<<endl;

    cout<<"-----------"<<endl;

    cout<<"tcw"<<tcw<<endl;

    delete mono_initializer;

    if(Rcw.data != NULL){

    cv::Mat pos02 = -Rcw.t()*tcw;  //第二副图原点在世界坐标系中的位置

//  pos02.convertTo(pos02, CV_32FC1);

    curr_->frame_pos_<< pos02.at<float>(0,0),pos02.at<float>(0,1),pos02.at<float>(0,2);

    Keyframes.push_back(curr_);

     }

}


void VisualOdometry::UndistortKeyPoints(){

    int N2 = keypoints_curr_.size();
    int N1 = keypoints_ref_.size();


    mK=camera_->mK.clone();

    mDistCoef=camera_->mDistCoef.clone();

    if(mDistCoef.at<float>(0)==0.0)
    {
       kpcurr_Un =keypoints_curr_;
       kpref_Un =keypoints_curr_;

        return;
    }

    // Fill matrix with points
    cv::Mat mat2;//(N2,2,CV_32F);
    cv::Mat mat1;//(N1,2,CV_32F);
    mat2 = Mat::zeros(N2,2,CV_32F);
    mat1 = Mat::zeros(N1,2,CV_32F);



    for(int i=0; i<N2; i++)
    {
        mat2.at<float>(i,0)=keypoints_curr_[i].pt.x;
        mat2.at<float>(i,1)=keypoints_curr_[i].pt.y;
    }

    for(int i=0; i<N1; i++)
    {
        mat1.at<float>(i,0)=keypoints_ref_[i].pt.x;
        mat1.at<float>(i,1)=keypoints_ref_[i].pt.y;
    }
    cout<<mat2<<endl;
    // Undistort points
    mat2=mat2.reshape(2);
    mat1=mat1.reshape(2);
    cout<<mat2<<endl;

//    std::cout<<mK.at<float>(0,0)<<endl;
//    std::cout<<mDistCoef<<endl;

    cv::undistortPoints(mat2,mat2,mK,mDistCoef,cv::Mat(),mK);
    cv::undistortPoints(mat1,mat1,mK,mDistCoef,cv::Mat(),mK);
    cout<<mat2<<endl;

    mat2=mat2.reshape(1);
    mat1=mat1.reshape(1);
    cout<<mat2<<endl;


    // Fill undistorted keypoint vector
//    kpcurr_Un.reserve(N2);
//    kpref_Un.resize(N1);
        kpcurr_Un.clear();
        kpref_Un.clear();


    for(int i=0; i<N2; i++)
    {
        cv::KeyPoint kp = keypoints_curr_[i];
        kp.pt.x=mat2.at<float>(i,0);
        kp.pt.y=mat2.at<float>(i,1);
        //kpcurr_Un[i]=kp;
        kpcurr_Un.push_back(kp);
    }

    for(int i=0; i<N1; i++)
    {
        cv::KeyPoint kp = keypoints_ref_[i];
        kp.pt.x=mat1.at<float>(i,0);
        kp.pt.y=mat1.at<float>(i,1);
        //kpref_Un[i]=kp;
        kpref_Un.push_back(kp);
    }
}



bool VisualOdometry::addimage(const Mat& image){

    cv::imshow("image1", image);

    static long image_id = 0;
    image_id ++;
    switch ( state_ )
    {
    case NOT_INITIALIZED:
    {

#if ShowState

        cout<<"---------------------"<<endl;

        cout<<"state: NOT_INITIALIZED "<<endl;

#endif

        ref_ = Frame::createFrame(image,camera_);

        if(ref_->keypoints_.size()<100) return 0;

        ref_->UndistortKeyPoints();

        if(first_initialize){

        ref_->frame_pos_<<0,0,0;

        float b[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};

        ref_->pose=(Mat(4,4,CV_32F,b).clone());

                            }

        else{

            ref_->frame_pos_=saved_frame_pos_;

            ref_->pose=saved_pose.clone();

        }

        state_ = INITIALIZING;

        break;
    }

    case OK:
    {

        first_initialize = 0;

#if ShowState

        cout<<"---------------------"<<endl;

        cout<<"state: OK "<<endl;

#endif

        curr_ = Frame::createFrame(image,camera_);

        if(curr_->keypoints_.size() < 100)
        {
            if (image_id - ref_->id_>10) state_ = LOST;

            return 0;
        }

        Monocular_SLEMatching();  //线段端点匹配

        if (feature_matches_.size()<50) {

            if (image_id - ref_->id_>10) state_ = LOST;

            return 0;
        }

        cv:Mat K= camera_->mK;

        curr_->UndistortKeyPoints();

        mono_initializer = new initialize_mono(ref_->Unkeypoints,curr_->Unkeypoints,feature_matches_ , 1,100, K);

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        mono_initializer->Initialize(Rcw,tcw,mvIniP3D,vbTriangulated);

        cout<<"Rcw:"<<Rcw<<endl;

        cout<<"-----------"<<endl;

        cout<<"tcw:"<<tcw<<endl;

        delete mono_initializer;

        if(Rcw.data != NULL){
            cur_t = cur_t + (cur_R*tcw);
            cur_R = Rcw*cur_R;

          float b[16]={Rcw.at<float>(0,0),Rcw.at<float>(0,1),Rcw.at<float>(0,2),tcw.at<float>(0),
                         Rcw.at<float>(1,0),Rcw.at<float>(1,1),Rcw.at<float>(1,2),tcw.at<float>(1),
                         Rcw.at<float>(2,0),Rcw.at<float>(2,1),Rcw.at<float>(2,2),tcw.at<float>(2),
                         0,0,0,1};

         Mat temp_pose = (Mat(4,4,CV_32F,b).clone());

         curr_->pose = temp_pose*ref_->pose;

         cout<<"curr_->pose:"<<curr_->pose<<endl;

         Mat R = (Mat_<float>(3,3)<< curr_->pose.at<float>(0,0),curr_->pose.at<float>(0,1),curr_->pose.at<float>(0,2),
                    curr_->pose.at<float>(1,0),curr_->pose.at<float>(1,1),curr_->pose.at<float>(1,2),
                    curr_->pose.at<float>(2,0),curr_->pose.at<float>(2,1),curr_->pose.at<float>(2,2));

         Mat T = (Mat_<float>(3,1)<<curr_->pose.at<float>(0,3),curr_->pose.at<float>(1,3),curr_->pose.at<float>(2,3));

         cout<<"T:"<<T<<endl;

         cout<<"R:"<<R<<endl;

         cv::Mat pos02 = -R.t()*T;  //第二副图原点在世界坐标系中的位置

         cout<<"位置:"<<pos02<<endl;

        curr_->frame_pos_<< pos02.at<float>(0,0),pos02.at<float>(0,1),pos02.at<float>(0,2);

        draw_trajactory();

        Keyframes.push_back(curr_);

        ref_ = curr_;

        }

        else      {

            curr_->frame_pos_=ref_->frame_pos_;

            curr_->pose=ref_->pose.clone();

            if(image_id - ref_->id_>10)     state_ = LOST;

            return 0;
          }

        break;
    }

    case INITIALIZING:
    {

#if ShowState

        cout<<"+-+-+-+-+-+-+-+-+-+-+-+-+-+"<<endl;

        cout<<"state: INITIALIZING "<<endl;

#endif

        curr_ = Frame::createFrame(image,camera_);

        if(curr_->keypoints_.size() < 100)
        {
            if (image_id - ref_->id_>5) state_ = NOT_INITIALIZED;

            return 0;

        }

        Monocular_SLEMatching();


        if (feature_matches_.size()<50) {

            if (image_id - ref_->id_>5) state_ = NOT_INITIALIZED;

            return 0;
        }

        cv::Mat K= camera_->mK;

        curr_->UndistortKeyPoints();

        mono_initializer = new initialize_mono(ref_->Unkeypoints,curr_->Unkeypoints,feature_matches_ , 1,100, K);

        cv::Mat Rcw; // Current Camera Rotation

        cv::Mat tcw; // Current Camera Translation

        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        mono_initializer->Initialize(Rcw,tcw,mvIniP3D,vbTriangulated);

        cout<<"Rcw"<<Rcw<<endl;

        cout<<"-----------"<<endl;

        cout<<"tcw"<<tcw<<endl;

        delete mono_initializer;

        if(Rcw.data != NULL){

            if(first_initialize){

        cur_R = Rcw.clone();

        cur_t = tcw.clone();
            }

        float b[16]={Rcw.at<float>(0,0),Rcw.at<float>(0,1),Rcw.at<float>(0,2),tcw.at<float>(0),
                     Rcw.at<float>(1,0),Rcw.at<float>(1,1),Rcw.at<float>(1,2),tcw.at<float>(1),
                     Rcw.at<float>(2,0),Rcw.at<float>(2,1),Rcw.at<float>(2,2),tcw.at<float>(2),
                     0,0,0,1};

         Mat temp_pose = (Mat(4,4,CV_32F,b).clone());

        curr_->pose = temp_pose*ref_->pose;

        cv::Mat pos02 = (Mat_<float>(3,1)<<curr_->pose.at<float>(0,3),curr_->pose.at<float>(1,3),curr_->pose.at<float>(2,3));


//        Mat R = (Mat_<float>(3,3)<< curr_->pose.at<float>(0,0),curr_->pose.at<float>(0,1),curr_->pose.at<float>(0,2),
//                curr_->pose.at<float>(1,0),curr_->pose.at<float>(1,1),curr_->pose.at<float>(1,2),
//                curr_->pose.at<float>(2,0),curr_->pose.at<float>(2,1),curr_->pose.at<float>(2,2));

//        Mat T = (Mat_<float>(3,1)<<curr_->pose.at<float>(0,3),curr_->pose.at<float>(1,3),curr_->pose.at<float>(2,3));

//        cv::Mat pos02 = -R.t()*T;  //第二副图原点在世界坐标系中的位置

        curr_->frame_pos_<< pos02.at<float>(0,0),pos02.at<float>(0,1),pos02.at<float>(0,2);

        state_ = OK;

        Keyframes.push_back(curr_);

        cout<<"keyframe_ID:"<<Keyframes[Keyframes.size()-1]->id_<<endl;

        ref_ = curr_;

        }
        else{

            curr_->frame_pos_=ref_->frame_pos_;

            curr_->pose=ref_->pose.clone();

            if(image_id - ref_->id_>5)     {

                state_ = NOT_INITIALIZED;
            }

            return 0;
          }
        break;

    }


    case LOST:        
    {
        saved_frame_pos_=ref_->frame_pos_;

        saved_pose=ref_->pose.clone();

        image_id = image_id-1;

        state_ = NOT_INITIALIZED;

        cout<<"keyframe.size:"<<Keyframes.size()<<endl;
#if ShowState

        cout<<"+-+-+-+-+-+-+-+-+-+-+-+-+-+"<<endl;

        cout<<"state: LOST "<<endl;

#endif
    }
    }

    return true;
}



bool VisualOdometry::draw_trajactory(){
    cv::circle(track,cv::Point(500+5*cur_t.at<float>(0),500+5*cur_t.at<float>(2)),1.5,cv::Scalar(0,0,0),1);
    imshow("trajectory",track);

}
}
