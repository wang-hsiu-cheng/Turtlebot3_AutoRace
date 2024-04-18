#ifndef _WHEEL_H_ // could this be edited?
#define _WHEEL_H_ // I M Afraiiiiid!
// yes! this has to be changed. or the header file may not be define. by TWang
// ******miffy edited this@0229. miffy dont know anything.
#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h> //miffy changed from Point to Twist.
#include "nav_msgs/Odometry.h"
#include "yaml-cpp/yaml.h"

ros::Publisher wheel_publisher; // Topic: /cmd_vel
geometry_msgs::Twist wheel_pub;

ros::Subscriber wheel_subscriber; // Topic: mecanum_fromSTM
geometry_msgs::Twist wheel_sub;

// namespace WHEEL is to distinguish it from IMU
namespace WHEEL
{
    bool data_check;
    void init(ros::NodeHandle nh);
    void callback(const nav_msgs::Odometry::ConstPtr &msg);

    int stop();
    int move_front(int mode, float angleRad);
    int moveStraightLine(float distance);
    int moveTo(double distance, double angleRad);

    /* param */
    int pub_x = 0;
    int pub_y = 0;
    int pub_z = 0;
    /* roslaunch param */
    double rad_p_control_0 = 0.1745329252;
    double rad_p_control_1 = 0.5235988;
    double rad_p_control_2 = 1.0471976;
    double rad_p_control_3 = 1.57079632679;
    double omega_p_control_0 = 0.087266462599716;
    double omega_p_control_1 = 0.174532925199433;
    double omega_p_control_2 = 0.349065850398866;
    double omega_p_control_3 = 0.523598775598299;
    double distance_p_control_0 = 0.03;
    double distance_p_control_1 = 0.1;
    double distance_p_control_2 = 0.2;
    double distance_p_control_3 = 0.5;
    double velocity_p_control_0 = 0.1;
    double velocity_p_control_1 = 0.35;
    double velocity_p_control_2 = 0.5;
    double velocity_p_control_3 = 0.7;
    double velocity_p_control_4 = 0.5;
}

#endif
