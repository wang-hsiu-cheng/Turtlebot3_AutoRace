#ifndef _DETECT_H_
#define _DETECT_H_

#include "race/vision.h"
#include "race/roadCamera.h"
#include "race/wheel.h"
#include "ros/ros.h"
#include <iostream>
#include "std_msgs_Float32.h"
#include <math.h>

ros::Subscriber vl53_subscriber; // Topic: mecanum_fromSTM
std_msgs::Float32 vl53_sub;

// #define stage1_yaml "/home/Turtlebot_AutoRace/src/race/path/stage1.yaml"
namespace DETECT
{
    void init(ros::NodeHandle nh);
    void callback(const std_msgs::Float32::ConstPtr &msg);

    void runAndDetectImage(const int);
    int fanceDetect(void);
    int turnSignDetect(void);
    void positionCheck(void);


}

#endif