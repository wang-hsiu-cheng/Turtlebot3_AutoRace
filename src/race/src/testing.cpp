#include <iostream>
#include "ros/ros.h"
#include "race/detect.h"
#include "race/wheel.h"
#include "race/vision.h"
#include "race/roadCamera.h"
// #define PI 3.141592653589793238462643383279502884

using namespace std;

void init_all_sensors(ros::NodeHandle nh)
{
    WHEEL::init(nh);
    // VISION::init(nh);
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "testing");
    ros::NodeHandle nh;
    init_all_sensors(nh);

    int test_phase = -1;
    int sign_number;
    double speed, angle, distance, goal_x, goal_y;
    // nh.getParam("test_phase", test_phase);
    while (test_phase != 0)
    {
        cout << "input test number: \n";
        cin >> test_phase;
        switch (test_phase)
        {
        case 1:
            printf("enter distance and angle: ");
            cin >> distance >> angle;
            angle = angle / 180 * PI;
            WHEEL::moveTo(distance, angle);
            break;

        case 2:
            printf("enter angle: ");
            cin >> angle;
            angle = angle / 180 * PI;
            WHEEL::move_front(0, angle);
            break;

        case 3:
            printf("enter distance: ");
            cin >> distance;
            WHEEL::moveStraightLine(distance);
            break;

        case 4:
            printf("detect, enter picture code: ");
            cin >> sign_number;
            VISION::takingPhoto(sign_number);
            cout << VISION::isDetected << endl;
            break;

        case 5:
            printf("detect and run, enter picture code: ");
            cin >> sign_number;
            DETECT::runAndDetectImage(sign_number);
            break;
        // case 6:
        //     printf("navigation\n"); //, enter goal x and goal y:
        //     // cin >> goal_x >> goal_y;
        //     std::string launchFilePath = "~/Turtlebot3_AutoRace/src/tb3_navigation/launch/move_base.launch"; // Replace with the path to your actual launch file
        //     std::string command = "roslaunch " + launchFilePath;
        //     int result = system(command.c_str());
        //     if (result == -1)
        //     {
        //         ROS_ERROR("Failed to execute roslaunch command");
        //     }
        //     break;

        default:
            break;
        }
    }
    return 0;
}
