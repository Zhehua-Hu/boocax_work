#ifndef BARCODE_LOCALIZATION_H
#define BARCODE_LOCALIZATION_H
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;
bool barcode_localization(Mat srcImage, vector<Point2f>& pt);
void precise_localization(Mat srcImage, vector<Point2f>& pt, int radius);
bool barcode_veterx_localization(Mat bar_photo, vector<Point2f>& pts2d);

float cal_distance(const Mat srcImg,const Mat temp);
#define test 2
#define test_pnp (test == 1)
#define test_barcode (test == 2)



#endif // BARCODE_LOCALIZATION_H
