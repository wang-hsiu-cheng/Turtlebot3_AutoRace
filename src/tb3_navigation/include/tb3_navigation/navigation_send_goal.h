#ifndef _SEND_GOAL_H_
#define _SEND_GOAL_H_

#define goal_yaml "/home/ditrobotics/Turtlebot3_AutoRace/src/tb3_navigation/params/path.yaml"

double tf_x;
double tf_y;
double tf_theta;
double tf_thetaAngle;
double x = 1.0;
double y = 0.0;
double theta = 1.0;
// bool goalReached = false;

int SendGoal(double x, double y);

#endif