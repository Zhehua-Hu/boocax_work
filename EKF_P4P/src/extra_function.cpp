#include "extra_function.h"
cv::Vec2f cal_variance(std::vector<cv::Vec2f> num)
{  //计算方差,输入数据第一个是标准值
    cv::Vec2f var(0.0,0.0);
    for (int i =1 ; i<num.size(); i++){
        var[0]+=(num[i]-num[0])[0]*(num[i]-num[0])[0];
        var[1]+=(num[i]-num[0])[1]*(num[i]-num[0])[1];
    }
    var[0] = var[0]/(num.size()-1);
    var[1] = var[1]/(num.size()-1);

    return var;
}

float cal_variance(std::vector<float> num){  //计算方差,输入数据第一个是标准值
    float var=(num[1]-num[0])*(num[1]-num[0]);

    for (int i =2 ; i<num.size(); i++){
        var+=(num[i]-num[0])*(num[i]-num[0]);
    }
    return var/(num.size()-1);
}



cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f theta)
{
    // 计算旋转矩阵的X分量
    cv::Mat R_x = (cv::Mat_<float>(3,3) <<
               1,       0,              0,
               0,       cos(theta[0]),   sin(theta[0]),
               0,       -sin(theta[0]),   cos(theta[0])
               );

    // 计算旋转矩阵的Y分量
    cv::Mat R_y = (cv::Mat_<float>(3,3) <<
               cos(theta[1]),    0,      -sin(theta[1]),
               0,               1,      0,
               sin(theta[1]),   0,      cos(theta[1])
               );

    // 计算旋转矩阵的Z分量
    cv::Mat R_z = (cv::Mat_<float>(3,3) <<
               cos(theta[2]),    sin(theta[2]),      0,
               -sin(theta[2]),    cos(theta[2]),       0,
               0,               0,                  1);

    // 合并
//    cv::Mat Rotation_matrix = (R_z * R_y * R_x).clone();

    return R_z*R_y*R_x;
}

cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)  //X-Y-Z 顺序
{

    float sy = sqrt(R.at<float>(0,0) * R.at<float>(0,0) +  R.at<float>(1,0) * R.at<float>(1,0) );

    bool singular = sy < 1e-6; //

    float x, y, z;
    if (!singular)
    {
        x = atan2(-R.at<float>(2,1) , R.at<float>(2,2));
        y = asin( R.at<float>(2,0));
        z = atan2(-R.at<float>(1,0), R.at<float>(0,0));
    }
    else
    {
        x = asin(-R.at<float>(1,2)/R.at<float>(1,1));
        y = asin( R.at<float>(2,0));
        z = 0;
    }

    float y2 = asin( R.at<float>(2,0));
    y=(float) y2;
    return cv::Vec3f(-3.1415/2, y, 0);
}

cv::Mat double2float(const cv::Mat M){
    int row = M.rows,col = M.cols;
    cv::Mat fM = cv::Mat::eye(row,col,CV_32F);
    for(int i =0;i<row;i++){
        for (int j=0;j<col;j++)
            fM.at<float>(i,j)=(float)M.at<double>(i,j);
    }
    return fM;
}

float norm(Vec3f a){
    return sqrt(a.dot(a));
}

bool collineation(cv::Vec3f p1, cv::Vec3f p2,cv::Vec3f p){
    float e = 0.0001;
    cv::Vec3f direction1 = p-p1;
    cv::Vec3f direction2 = p-p2;
    if(direction1.dot(direction1)<e) return 1;
    else if (direction2.dot(direction2)<e) return 1;
    cv::Vec3f cross = direction1.cross(direction2);
    float sin2angle = cross.dot(cross)/(direction1.dot(direction1)*direction2.dot(direction2));
    if(sin2angle<e) return 1;
    return 0;
}

void check_yaw(float& yaw){
//    if(yaw>=90) yaw = yaw-90;
//    else if(yaw <=0)  yaw = yaw+360;
}

