#ifndef SOLVE_PNP_H
#define SOLVE_PNP_H
#include <iostream>
#include <opencv2/opencv.hpp>
//#include <pangolin/pangolin.h>

using namespace cv;
using namespace std;

void solve_pnp(vector<Point3f> objectPoints, vector<Point2f> imagePoints,
               InputArray cameraMatrix, InputArray distCoeffs,
               bool useExtrinsicGuess = false, int flags = SOLVEPNP_ITERATIVE,int radius = 1);

void draw_camera(vector<Point3f> objectPoints, vector<vector<double>> pose_fin);

#endif // SOLVE_PNP_H
