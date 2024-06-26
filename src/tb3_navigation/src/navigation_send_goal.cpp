#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>     // get the transtorm between map and base_link
#include <geometry_msgs/TransformStamped.h> // get the transtorm between map and base_link
#include <tf2/exceptions.h>                 // get the transtorm between map and base_link
#include "tb3_navigation/navigation_send_goal.h"
#include <actionlib/client/simple_action_client.h> // receive goal callback
#include <move_base_msgs/MoveBaseAction.h>         // goal msgs

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void Init(ros::NodeHandle nh_send_goal)
{
    nh_send_goal.getParam("navigation_send_goal/goal_x", x);
    nh_send_goal.getParam("navigation_send_goal/goal_y", y);
    nh_send_goal.getParam("navigation_send_goal/goal_theta", theta);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_send_goal");
    ros::NodeHandle nh_send_goal;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    // ROS_INFO("goal node running\n");
    // MoveBaseClient ac("move_base", true);
    // while (!ac.waitForServer(ros::Duration(5.0)))
    // {
    //     ROS_INFO("waiting for move_base server come up");
    // }
    Init(nh_send_goal);

    ROS_INFO("Get current position\n");
    geometry_msgs::TransformStamped transformStamped;
    transformStamped = tfBuffer.lookupTransform("map", "base_footprint", ros::Time(0));
    tf_x = transformStamped.transform.translation.x;
    tf_y = transformStamped.transform.translation.y;
    tf_theta = transformStamped.transform.rotation.w;
    printf("tf_x: %f, tf_y: %f tf_theta: %f\n", tf_x, tf_y);

    YAML::Node pathConfig = LoadFile(goal_yaml);
    for (auto point : pathConfig)
    {
        x = point["da"][0].as<double>();
        y = point["da"][1].as<double>();
        SendGoal(x, y);
    }
    // ROS_INFO("Start send goal\n");
    // move_base_msgs::MoveBaseGoal goal;
    // goal.target_pose.header.frame_id = "map";
    // goal.target_pose.header.stamp = ros::Time::now();
    // goal.target_pose.pose.position.x = tf_x + x;
    // goal.target_pose.pose.position.y = tf_y + y;
    // goal.target_pose.pose.orientation.w = tf_theta + theta;
    // goal.target_pose.pose.orientation.x = transformStamped.transform.rotation.x;
    // goal.target_pose.pose.orientation.y = transformStamped.transform.rotation.y;
    // goal.target_pose.pose.orientation.z = transformStamped.transform.rotation.z;
    // ac.sendGoal(goal);
    // ROS_INFO("send Nav end\n");

    // ac.waitForResult();
    // if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    // {
    //     printf("yes!");
    //     // std::string command = "^C ";
    //     // int result = system(command.c_str());
    //     return 1;
    // }
    // else
    // {
    //     ROS_INFO("no");
    //     return 0;
    // }
}
int SendGoal(double _x, double _y)
{
    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("waiting for move_base server come up");
    }
    x = _x;
    y = _y;

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = tf_x + x;
    goal.target_pose.pose.position.y = tf_y + y;
    goal.target_pose.pose.orientation.w = tf_theta + theta;

    ac.sendGoal(goal);

    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        printf("yes!");
        return 1;
    }
    else
    {
        printf("no!");
        return 0;
    }
}