#include "practice/odom.h"
#include <math.h>

using namespace std;

Odom::Odom(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
    initialize();
}

bool Odom::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    bool prev_active = p_active_;

    /* get param */
    nh_local_.param<bool>("active", p_active_, true);
    nh_local_.param<bool>("publish_odom", p_publish_odom_, true);

    nh_local_.param<double>("frequency", p_frequency_, 30);

    nh_local_.param<double>("cov_vx", p_cov_vx_, 1e-9);
    nh_local_.param<double>("cov_vy", p_cov_vy_, 1e-9);
    nh_local_.param<double>("cov_vyaw", p_cov_vyaw_, 1e-9);

    nh_local_.param<string>("fixed_frame_id", p_fixed_frame_id_, "odom");
    nh_local_.param<string>("target_frame_id", p_target_frame_id_, "base_footprint");

    if (p_active_ != prev_active) {

        if (p_active_) {
            twist_sub_ = nh_.subscribe("cmd_vel", 10, &Odom::twistCallback, this);
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
        }
        else {
            twist_sub_.shutdown();
            odom_pub_.shutdown();
        }
    }


    output_odom_.header.frame_id = p_fixed_frame_id_;
    output_odom_.child_frame_id = p_target_frame_id_;

    output_odom_.pose.pose.position.x = 0;
    output_odom_.pose.pose.position.y = 0;

    output_odom_.pose.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                                    //x,         y,   z,   p,   r,   y
    output_odom_.twist.covariance = {p_cov_vx_, 0.0,       0.0, 0.0, 0.0, 0.0,
                                    0.0,       p_cov_vy_, 0.0, 0.0, 0.0, 0.0,
                                    0.0,       0.0,       0.0, 0.0, 0.0, 0.0,
                                    0.0,       0.0,       0.0, 0.0, 0.0, 0.0,
                                    0.0,       0.0,       0.0, 0.0, 0.0, 0.0,
                                    0.0,       0.0,       0.0, 0.0, 0.0, p_cov_vyaw_};

    return true;
}

void Odom::twistCallback(const geometry_msgs::Twist::ConstPtr& ptr) {

    // TODO-2
    // Put velocity ( twist ) into output_odom_ -> twist
    output_odom_.twist.twist.linear = ptr->linear;
    output_odom_.twist.twist.angular = ptr->angular;
    // Put localization into output_odom_ -> pose
    output_odom_.pose.pose.position.x += ptr->linear.x * (float)cos(ptr->angular.z) / 30;
    output_odom_.pose.pose.position.y += ptr->linear.x * (float)sin(ptr->angular.z) / 30;
    output_odom_.pose.pose.orientation.w = (float)sqrt(2 * cos(ptr->angular.z) + 2) / 2;
    output_odom_.pose.pose.orientation.x = 0;
    output_odom_.pose.pose.orientation.y = 0;
    output_odom_.pose.pose.orientation.z = (float)(sin(ptr->angular.z) / sqrt(2 * cos(ptr->angular.z) + 2));
    // You need use integral to get the localization

    publish();
}

void Odom::publish() {
    if (p_publish_odom_)
        odom_pub_.publish(output_odom_);
}