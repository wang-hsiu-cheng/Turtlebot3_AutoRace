#ifndef _RUN_H_
#define _RUN_H_

#include "race/detect.h"
#include "race/turn_left_right.h"
#include "race/parking.h"

#include "race/vision.h"
#include "race/wheel.h"

int init_all_sensors(ros::NodeHandle nh);
void race_levels(const int, const int, ros::NodeHandle nh);
int navigationSystem(ros::NodeHandle nh);

bool data_check = false;
bool goalReached = false;
int reset_state;
int begin_state;
int level = 0;

#endif