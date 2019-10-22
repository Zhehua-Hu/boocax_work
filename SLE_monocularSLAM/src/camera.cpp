#include "./include/camera.h"
#include "./include/config.h"

namespace SLE_monocularSLAM{

Camera::Camera(){
        fx_ = Config::get<float>("camera.fx");
        fy_ = Config::get<float>("camera.fy");
        cx_ = Config::get<float>("camera.cx");
        cy_ = Config::get<float>("camera.cy");

        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = fx_;
        K.at<float>(1,1) = fy_;
        K.at<float>(0,2) = cx_;
        K.at<float>(1,2) = cy_;
        K.copyTo(mK);

        float k1 = Config::get<float>("camera.k1");
        float k2 = Config::get<float>("camera.k2");
        float k3 = Config::get<float>("camera.k3");


        cv::Mat DistCoef(5,1,CV_32F);
        DistCoef.at<float>(0) = k1;
        DistCoef.at<float>(1) = k2;
        DistCoef.at<float>(2) = 0;
        DistCoef.at<float>(3) = 0;
        DistCoef.at<float>(4) = k3;

        DistCoef.copyTo(mDistCoef);



        depth_scale_ = Config::get<float>("camera.depth_scale");

}


Vector3d Camera::world2camera(const Vector3d &p_w, const SE3 &T_c_w){
    return T_c_w * p_w;
}

Vector3d Camera::camera2world(const Vector3d &p_c, const SE3 &T_c_w){
    return T_c_w.inverse()*p_c;
}

Vector2d Camera::camera2pixel(const Vector3d &p_c){
    return Vector2d(
                fx_*p_c(0,0)/p_c(2,0)+cx_,
                fy_*p_c(1,0)/p_c(2,0)+cy_
                );
}

Vector3d Camera::pixel2camera(const Vector2d &p_p, double depth)
{
    return Vector3d(
                (p_p(0,0)-cx_)*depth/fx_,
                (p_p(1,0)-cy_)*depth/fy_,
                depth
                );
}

Vector2d Camera::world2pixel(const Vector3d &p_w, const SE3 &T_c_w){
    return camera2pixel(world2camera(p_w,T_c_w));
}

Vector3d Camera::pixel2world(const Vector2d &p_p, const SE3 &T_c_w, double depth){
    return camera2world(pixel2camera(p_p),T_c_w);
}


}
