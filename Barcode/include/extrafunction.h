#ifndef EXTRAFUNCTION_H
#define EXTRAFUNCTION_H
#include <opencv2/opencv.hpp>
using namespace cv;

Mat eulerAnglesToRotationMatrix(Vec3f &theta);
Vec3f rotationMatrixToEulerAngles(Mat &R);


#endif // EXTRAFUNCTION_H
