#ifndef _VISION_H_
#define _VISION_H_

#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#define PI 3.1415926536

using namespace std;
using namespace cv;

namespace VISION
{

    // int lastImageName = 100;
    int imageName = 100;
    int counter = 0;
    const int detectingLoop = 5;
    int detectedCounter = 0;
    int detectTime = 0;
    bool isDetected = false;
    int rightCount = 0;
    int leftCount = 0;
    char direction = 'n';
    int downCount = 0;
    int riseCount = 0;
    bool isRise = true;

    void init(ros::NodeHandle nh);
    void takingPhoto(int imageName);
    void DontDetectAnything();
    void greenLightImage(Mat original_image, Mat image);
    void warnSignImage(Mat original_image, Mat image);
    void turnSignImage(Mat original_image, Mat image);
    void stop_sign_image();
    void parking_sign_image();
    void fanceImage(Mat original_image, Mat image);
    Mat filtGraph(Mat img, int color_code);
    enum PhotoColor
    {
        RED1,
        RED2,
        BLUE,
        GREEN,
        YELLOW
    };
    /* launch param*/
    int hue_m = 0;
    int hue_M = 255;
    int sat_m = 0;
    int sat_M = 255;
    int val_m = 0;
    int val_M = 255;

    double slop = 0;
}

#endif
