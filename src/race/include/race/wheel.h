#ifndef _WHEEL_H_ // could this be edited?
#define _WHEEL_H_ // I M Afraiiiiid!
// yes! this has to be changed. or the header file may not be define. by TWang
// ******miffy edited this@0229. miffy dont know anything.
#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h> //miffy changed from Point to Twist.
#include "yaml-cpp/yaml.h"

ros::Publisher wheel_publisher; // Topic: mecanum_toSTM
geometry_msgs::Twist wheel_pub;

ros::Subscriber wheel_subscriber; // Topic: mecanum_fromSTM
geometry_msgs::Twist wheel_sub;

// namespace WHEEL is to distinguish it from IMU
namespace WHEEL
{
    bool data_check;
    void init(ros::NodeHandle nh);
    void callback(const geometry_msgs::Twist::ConstPtr &msg);

    // void move(vector<double> point_left, vector<double> vector_left, vector<double> point_right, vector<double> vector_right);
    void stop();
    void move_front(int mode, float angle_rad);
    void moveTo(double x_cor, double y_cor, double z_cor);
    void moveUP(double x_cor, double y_cor, double z_cor);

    void readPath(std::string yaml_path);

    /* param */
    int pub_x = 0;
    int pub_y = 0;
    int pub_z = 0;

    /* roslaunch param */
    double calibration_x_intercept = 0.7669;
    double calibration_y_intercept = -0.0927;
    double calibration_z_intercept = 0;
    double calibration_x = 6.0207;
    double calibration_y = 6.408;
    double calibration_z = 6.3;

    double z_overcali_mode = false;

    double max_xy = 2;
    double min_xy = 0.01;
    double max_z = 0.15;
    double acc_xy = 0.02;
    double acc_zz = 0.0008;
    double maxUP_xy = 6;
    double accUP_xy = 0.03;

    double kp = 0.8;
    double fod_xy = 0.2; // fraction of deceleration: start decelerate at the last dr of the whole distance
    double fod_z = 0.02;
    double kp_xy = 0.8; // p gain for x- y-direction control
    double kp_z = 0.8;  // p gain for z-direction control

    double x_tol_margin = 0.7;   // x tolerance critical value
    double y_tol_margin = 0.7;   // y tolerance critical value
    double z_tol_margin = 0.001; // z tolerance critical value
}

#endif
