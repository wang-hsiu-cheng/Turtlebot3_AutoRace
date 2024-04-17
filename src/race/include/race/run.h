#ifndef _RUN_H_
#define _RUN_H_

#include "race/wheel.h"

#define PI 3.1415926536
#define road1_yaml "~Turtlebot3_AutoRace/src/race/path/road1.yaml"
#define road2R_yaml "/home/twang/Turtlebot3_AutoRace/src/race/path/road2R.yaml"
#define road2L_yaml "/home/twang/Turtlebot3_AutoRace/src/race/path/road2L.yaml"
#define road5_yaml "/home/twang/Turtlebot3_AutoRace/src/race/path/road5.yaml"
#define road6_yaml "/home/twang/Turtlebot3_AutoRace/src/race/path/road6.yaml"

using namespace YAML;

double distance, angle, angleRad;

void run1();
void run2(char direction);
void run5();
void run6();
void run(YAML::Node);

#endif