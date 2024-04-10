#ifndef _RUN_H_
#define _RUN_H_

#include "race/wheel.h"

#define PI 3.1415926536
#define road1_yaml "../../path/road1.yaml"
#define road2R_yaml "../../path/road2R.yaml"
#define road2L_yaml "../../path/road2L.yaml"
#define road5_yaml "../../path/road5.yaml"
#define road6_yaml "../../path/road5.yaml"

using namespace YAML;

double distance, angle, angleRad;

void run1();
void run2(char direction);
void run5();
void run6();
void run(Node);

#endif