#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>         // get the transtorm between map and base_link
#include <geometry_msgs/TransformStamped.h>     // get the transtorm between map and base_link
#include <tf2/exceptions.h>                     // get the transtorm between map and base_link
#include "tb3_navigation/navigation_send_goal.h"
#include <actionlib/client/simple_action_client.h> // receive goal callback
#include <move_base_msgs/MoveBaseAction.h>         // goal msgs

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_send_goal");
    ros::NodeHandle nh_send_goal;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ROS_INFO("goal node running\n");

    nh_send_goal.getParam("/goal_x", x);
    nh_send_goal.getParam("/goal_y", y);
    nh_send_goal.getParam("/goal_theta", theta);

    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("waiting for move_bae server come up");
    }

    ROS_INFO("Get current position\n");
    geometry_msgs::TransformStamped transformStamped;
    transformStamped = tfBuffer.lookupTransform("base_link", "map", ros::Time(0));
    tf_x = -transformStamped.transform.translation.x;
    tf_y = -transformStamped.transform.translation.y;
    tf_theta = -transformStamped.transform.rotation.w;
    printf("tf_x: %g, tf_y: %g tf_theta: %g\n", tf_x, tf_y, tf_theta);
    ROS_INFO("Start send goal\n");
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = tf_x + x;
    goal.target_pose.pose.position.y = tf_y + y;
    goal.target_pose.pose.orientation.w = 1.0;
    ac.sendGoal(goal);
    ROS_INFO("send Nav end\n");

    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        printf("yes!");
        // std::string command = "^C ";
        // int result = system(command.c_str());
        return 1;
        // goalReached = true;
    }
    else
    {
        ROS_INFO("no");
        return 0;
        // goalReached = false;
    }

    // if (goalReached)
    //     return 0;
    // else if (!goalReached)
    //     return 1;
}