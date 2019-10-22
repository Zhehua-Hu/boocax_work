#ifndef COMMON_H
#define COMMON_H
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/ximgproc.hpp>
#include <vector>
#include <random>
#include "extra_function.h"
#include "vehicle.h"
#include "system.h"
#include "camera.h"
#include <mutex>
#include <thread>
#include <fstream>


#define Debug 1
#define test_PNP 1

extern ofstream OutFile;

#endif // COMMON_H
