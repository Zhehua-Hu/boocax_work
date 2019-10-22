#ifndef EXTRA_FUNCTION_H
#define EXTRA_FUNCTION_H
#include "common.h"

float cal_variance(std::vector<float> num);

cv::Vec2f cal_variance(std::vector<cv::Vec2f> num);

cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f theta);

cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R);

struct new_pose{
    bool isnewone=0;
    cv::Vec3f cam_pos= cv::Vec3f(0,0,0);
    cv::Vec3f m_campos= cv::Vec3f(0,0,0);
};

cv::Mat double2float(const cv::Mat M);

void check_yaw(float& yaw);

float norm(cv::Vec3f a);

bool collineation(cv::Vec3f p1, cv::Vec3f p2,cv::Vec3f p);

#endif // EXTRA_FUNCTION_H
