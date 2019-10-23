#ifndef COMMON_H
#define COMMON_H
#include<cstdlib>
#include<iostream>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/ximgproc.hpp>
//#include <3rd_party/include/feature_detection.h>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "se3.h"
#include "so3.h"

//#include <3rd_party/sophus/se3.h>
//#include "3rd_party/sophus/se3.h"
//#include "3rd_party/sophus/so3.h"
#include "descriptor_custom.hpp"
#include "feature_detection.h"

#define RUN 1
#define Debug 1
#define TEST_VEDIO (RUN == 1)
#define TEST_CAMERA (RUN == 2)
#define TEST_PICTURE (RUN == 3)
#define FIND_TABLELEG (RUN == 4)
#define TEST_HARRIES (RUN == 5)
#define TEST_CALHIST (RUN == 6)
#define TEST_READFILE (RUN == 7)
#define FINDOUTLINE (RUN == 8)
#define ShowState 1
#define ShowMatch  1
#define TEST_IMAGEPROCESS 0
using namespace std;
using namespace cv;
using namespace Sophus;
#endif // COMMON_H
