#include "practice/odom.h"

// Search TODO-
// Odometry message reference http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
// rqt_robot_steering : 
//      Install : sudo apt install ros-noetic-rqt-robot-steering
//      Run : rosrun rqt_robot_steering rqt_robot_steering

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom");

    ros::NodeHandle nh("");

    // TODO-1
    // 1. Check what is ~ mean
    // represent a absolute location of the topic with its topic parent name
    // 2. Why we need a "private node handle" 
    // hint : where we use the nh_local in Odom class
    //we use nh_local in the private function of Odom class(bool updateParams)
    // Example : https://blog.csdn.net/weixin_44401286/article/details/112204903
    ros::NodeHandle nh_local("~");

    try {
        Odom odom(nh, nh_local);
        ROS_INFO("[Odom]]: Initializing node");

        ros::spin();
    }
    catch (const char* s) {
        ROS_FATAL_STREAM("[Odom]: " << s);
    }
    catch (...) {
        ROS_FATAL_STREAM("[Odom]: Unexpected error");
    }

    return 0;
}