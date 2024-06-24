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

    // WHEEL::moveTo(0.45, 0);
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
        ros::Rate loop_rate(100);
        do
        {
            VISION::takingPhoto(1); // green_light_image
            loop_rate.sleep();
        } while (!VISION::isDetected);
        run1();

        if (level >= end_state)
            return;
        level++;
    }
    printf("[ROS_INFO] level = %d\n", level);
    if (level == 2)
    {
        // runAndDetectImage((int)warningSign);
        char dir = DETECT::turnSignDetect();
        run2(dir);

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
        run5();
        DETECT::fanceDetect();
        // do
        // {
        //     VISION::takingPhoto((int)fance); // fance_image
        // } while (!VISION::isRise);
        run6();
        // runAndDetectImage((int)warningSign);

        if (level >= end_state)
            return;
        level++;
    }
    printf("[ROS_INFO] level = %d\n", level);
    if (level == 6)
    {
        DETECT::positionCheck();
        ROS_INFO("Start Nav\n");
        navigationSystem(nh);

        return;
    }
}

int init_all_sensors(ros::NodeHandle nh)
{
    WHEEL::init(nh);
    DETECT::init(nh);
    VISION::init(nh);
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
    return 0;
}