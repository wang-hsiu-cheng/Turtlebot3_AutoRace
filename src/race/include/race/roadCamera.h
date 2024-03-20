#ifndef _CAMERA1_H_
#define _CAMERA1_H_

#define PI 3.1415926536

#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

namespace CAMERA1
{
    bool isDetected = false;
    double slop = PI / 2;
    double newSlop = PI / 2;
    int hue_m = 0;
    int hue_M = 255;
    int sat_m = 0;
    int sat_M = 255;
    int val_m = 0;
    int val_M = 255;

    void detectRoad();
    void filtGraph(Mat src, Mat &filteredImg, char color);
    void roadLineImage(Mat src, Mat &yellowImg, Mat &whiteImg);
}

#endif