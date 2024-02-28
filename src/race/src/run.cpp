#include "race/run.h"
#include <cstdlib>                     // for system()
#include <geometry_msgs/PoseStamped.h> // goal publish msg
#include <actionlib>
#include <time.h>
#include <move_base_msgs/MoveBaseActionResult.h>

enum ImageName
{
    nothing,
    trafficLight,
    warningSign,
    turnSign,
    parkingSign,
    stopSign,
    fance,
    tunnelSign
};

void navigationSystemCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg)
{
    if (msg->status.status == 3)
    { // Status 3 indicates that the goal was reached
        ROS_INFO("Robot reached the goal!");
        goalReached = true; // Set the flag to true
    }
    else
    {
        ROS_INFO("Robot is still navigating...");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run");
    ros::NodeHandle nh;
    // ros::Publisher visionPub = nh.advertise<std_msgs::Int64>("camera", 10);
    // ros::Subscriber visionSub = nh.subscribe("run_fromCamera", 1, callback);

    nh.getParam("reset_state", reset_state);
    ROS_INFO("State Now: %d", reset_state);

    level = init_all_sensors(nh);

    if (reset_state == 0)
    {
        while (nh.ok())
            ros::spinOnce();
    }
    else
    {
        race_levels(reset_state);
    }

    return EXIT_SUCCESS;
}
// void Callback(const std_msgs::Int64::ConstPtr &msg)
// {
//     std::cout << "msg.data = " << msg->data << "\n";
//     data_check = msg->data;
// }

void race_levels(const int state)
{
    // while (ros::ok() && !data_check)
    // {
    //     visionPub.publish(1);
    //     ros::spinOnce();
    // }
    // data_check = !data_check;
    do
    {
        VISION::takingPhoto((int)trafficLight); // green_light_image
    } while (!VISION::isDetected);
    VISION::isDetected = !VISION::isDetected;
    runAndDetectImage((int)warningSign);

    level++;
    if (level >= state)
        return;

    turnScript();
    runAndDetectImage((int)warningSign);

    level++;
    if (level >= state)
        return;

    // avoid_wall_script();
    runAndDetectImage((int)parkingSign);

    level++;
    if (level >= state)
        return;

    // parking_script();

    level++;
    if (level >= state)
        return;

    runAndDetectImage((int)fance);
    // while (ros::ok() && !data_check)
    // {
    //     visionPub.publish(6);
    //     ros::spinOnce();
    // }
    // data_check = !data_check;
    do
    {
        VISION::takingPhoto((int)fance); // fance_image
    } while (!VISION::isDetected);
    VISION::isDetected = !VISION::isDetected;
    runAndDetectImage((int)tunnelSign);

    level++;
    if (level >= state)
        return;

    while (navigationSystem() == 0)
        break;
    runAndDetectImage((int)nothing);

    level++;
    if (level >= state)
        return;
}

int init_all_sensors(ros::NodeHandle nh)
{
    // WHEEL::init(nh);
    // IMU::init();
    return 1;
}

int navigationSystem()
{
    double x;
    double y;
    double theta;
    nh.getParam("/goal_x", x);
    nh.getParam("/goal_y", y);
    nh.getParam("/goal_theta", theta);

    // Call roslaunch command to run your launch file
    std::string launchFilePath = "~/Turtlebot3_AutoRace/src/tb3_navigation/launch/move_base.launch"; // Replace with the path to your actual launch file
    std::string command = "roslaunch " + launchFilePath;
    int result = system(command.c_str());
    if (result == -1)
    {
        ROS_ERROR("Failed to execute roslaunch command");
        return 1;
    }
    // 宣告 publisher
    ros::Publisher pubGoal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    // Subscribe to the feedback topic
    ros::Subscriber sub = nh.subscribe("/move_base/result", 10, goalStatusCallback);

    // 宣告一個 PoseStamped 訊息目標，並輸入終點資訊
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.orientation.z = theta;

    // publish 目標位置訊息
    pubGoal.publish(goal);

    // Spin until the goal is reached
    ros::Rate rate(1); // Adjust the rate as needed
    double runningTime = (double)clock() / CLOCKS_PER_SEC;
    double endRunningTime = 100 + runningTime;
    while (ros::ok() && !goalReached && (endRunningTime - runningTime) > 0)
    {
        ros::spinOnce();
        rate.sleep();
        runningTime = (double)clock() / CLOCKS_PER_SEC;
    }

    if (goalReached)
        return 0;
    else if (!goalReached)
        return 1;
}
