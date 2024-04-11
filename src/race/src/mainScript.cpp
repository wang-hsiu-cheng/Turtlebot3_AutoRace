#include "race/mainScript.h"
#include <cstdlib> // for system()
// #include <actionlib/client/simple_action_client.h> // receive goal callback
// #include <move_base_msgs/MoveBaseAction.h> // goal msgs

// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

enum ImageName
{
    nothing,
    trafficLight,
    warningSign,
    turnSign,
    fance,
    stopSign,
    parkingSign
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run");
    ros::NodeHandle nh;

    nh.getParam("reset_state", reset_state);
    nh.getParam("begin_state", begin_state);
    ROS_INFO("State Now: %d", reset_state);

    level = init_all_sensors(nh);

    if (reset_state == 0)
    {
        while (nh.ok())
            ros::spinOnce();
    }
    else
    {
        race_levels(begin_state, reset_state, nh);
    }

    return EXIT_SUCCESS;
}

void race_levels(const int begin_state, const int end_state, ros::NodeHandle nh)
{
    level = begin_state;
    printf("[ROS_INFO] level = %d\n", level);
    if (level == 1)
    {
        // do
        // {
        //     VISION::takingPhoto((int)trafficLight); // green_light_image
        // } while (!VISION::isDetected);
        // runAndDetectImage((int)warningSign);
        run1();

        if (level >= end_state)
            return;
        level++;
    }
    printf("[ROS_INFO] level = %d\n", level);
    if (level == 2)
    {
        // turnScript();
        // cout << "finish turn script\n";
        // runAndDetectImage((int)warningSign);
        turnSignDetect();
        run2('l');

        if (level >= end_state)
            return;
        level++;
    }
    printf("[ROS_INFO] level = %d\n", level);
    if (level == 3)
    {
        // avoid_wall_script();
        // runAndDetectImage((int)parkingSign);

        if (level >= end_state)
            return;
        level++;
    }
    printf("[ROS_INFO] level = %d\n", level);
    if (level == 4)
    {
        // parking_script();

        if (level >= end_state)
            return;
        level++;
    }
    printf("[ROS_INFO] level = %d\n", level);
    if (level == 5)
    {
        // runAndDetectImage((int)fance);
        // do
        // {
        //     VISION::takingPhoto((int)fance); // fance_image
        // } while (!VISION::isRise);
        // runAndDetectImage((int)warningSign);
        run5();
        fanceDetect();

        if (level >= end_state)
            return;
        level++;
    }
    printf("[ROS_INFO] level = %d\n", level);
    if (level == 6)
    {
        run6();
        positionCheck();
        ROS_INFO("Start Nav\n");
        navigationSystem(nh);
        // ROS_INFO("Stop Nav\n");
        // runAndDetectImage((int)nothing);

        return;
    }
}

int init_all_sensors(ros::NodeHandle nh)
{
    WHEEL::init(nh);
    // IMU::init();
    return 1;
}

int navigationSystem(ros::NodeHandle nh)
{
    // Call roslaunch command to run your launch file
    std::string launchFilePath = "~/Turtlebot3_AutoRace/src/tb3_navigation/launch/move_base.launch"; // Replace with the path to your actual launch file
    std::string command = "roslaunch " + launchFilePath;
    int result = system(command.c_str());
    if (result == -1)
    {
        ROS_ERROR("Failed to execute roslaunch command");
        return 1;
    }

    // if (goalReached)
    //     return 0;
    // else if (!goalReached)
    //     return 1;
}