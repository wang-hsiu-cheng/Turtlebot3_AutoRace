#pragma once

#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

class Odom {

public:
    Odom(ros::NodeHandle& nh, ros::NodeHandle& nh_local);

private:

    void initialize() {
        std_srvs::Empty empt;
        updateParams(empt.request, empt.response);
    }

    bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void twistCallback(const geometry_msgs::Twist::ConstPtr& ptr);


    void publish();

    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::ServiceServer params_srv_;

    ros::Subscriber twist_sub_;
    ros::Publisher odom_pub_;

    nav_msgs::Odometry output_odom_;

    bool p_active_;
    bool p_publish_odom_;

    double p_frequency_;
    double p_cov_vx_;
    double p_cov_vy_;
    double p_cov_vyaw_;

    std::string p_fixed_frame_id_;
    std::string p_target_frame_id_;
};