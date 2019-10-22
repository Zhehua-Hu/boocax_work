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

#ifndef FRAME_H
#define FRAME_H

#include "common.h"
#include "camera.h"
#include "sift_line_endpoint.h"

namespace SLE_monocularSLAM
{
    
// forward declare 
class MapPoint;
class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long                  id_;         // id of this frame
    double                         time_stamp_; // when it is recorded
    SE3                            T_c_w_;      // transform from world to camera
    Camera::Ptr                    camera_;     // Pinhole RGBD Camera model 
    Mat                            color_, depth_, gray_; // color and depth image
    std::vector<cv::KeyPoint>      keypoints_;  // key points in image
    std::vector<cv::KeyPoint>      Unkeypoints;
    // std::vector<MapPoint*>         map_points_; // associated map points
    bool                           is_key_frame_;  // whether a key-frame
    int N=0;
    std::vector<cv::KeyPoint> mvKeysUn;

    Mat pose;

    Vector3d    frame_pos_;       // Frame Position in world

    static long unsigned int nNextId;

    long unsigned int mnId;

    cv::Mat mDistCoef;

    cv::Sift_Line_Endpoint::Ptr_SLE sle_frame;

    cv::Mat descriptors_;

public: // data members 
    Frame();
    Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat() );
    Frame( long id, const Mat& gray,Camera::Ptr camera=nullptr,double time_stamp=0, SE3 T_c_w=SE3(),  Mat color=Mat(), Mat depth=Mat() );
    ~Frame();
    
    static Frame::Ptr createFrame(const Mat& gray,Camera::Ptr camera);

    static Frame::Ptr createFrame();
    
    // find the depth in depth map
    double findDepth( const cv::KeyPoint& kp );

    void UndistortKeyPoints();
    
    // Get Camera Center
    Vector3d getCamCenter() const;
    
    void setPose( const SE3& T_c_w );
    
    // check if a point is in this frame 
    bool isInFrame( const Vector3d& pt_world );
};

}

#endif // FRAME_H
