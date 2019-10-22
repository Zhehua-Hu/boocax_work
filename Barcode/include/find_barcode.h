#ifndef FIND_BARCODE_H
#define FIND_BARCODE_H
#include <vector>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include "barcode_localization.h"
#include "extrafunction.h"
using namespace std;
using namespace cv;


struct barcode_position{
    Point2f center;
    float width;
    float height;
};
void para_initialize(int mode);
void task_over();
void find_manual(Mat img);
void On_mouse(int event, int x, int y, int flags, void*);//每次点击左键，将将当前点坐标存储到txt文件中，并在相应位置画红点
void manual_handlephoto (string p);

vector<string> getFiles(string cate_dir);
void readTxt(string file , vector<barcode_position>& barcode_positions);
void handlephoto (string p);
void handlebarcode(barcode_position l, Mat img );

extern ofstream OutFile;


#endif // FIND_BARCODE_H
