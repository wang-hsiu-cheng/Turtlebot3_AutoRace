#ifndef _ROAD_H_
#define _ROAD_H_

#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

namespace CAMERA1
{
    bool isDetected = false;
    int hue_m = 0;
    int hue_M = 255;
    int sat_m = 0;
    int sat_M = 255;
    int val_m = 0;
    int val_M = 255;

    void detectRoad();
    Mat filtGraph(Mat img);
    void roadLineImage(Mat src, Mat &ROI, bool isPrinted);
}

#endif