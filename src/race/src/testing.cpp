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
    VISION::init(nh);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "testing");
    ros::NodeHandle nh;
    init_all_sensors(nh);

    int test_phase = -1;
    int sign_number;
    double speed, angle;
    // nh.getParam("test_phase", test_phase);
    while (test_phase != 0)
    {
        cout << "input test number: \n";
        cin >> test_phase;
        switch (test_phase)
        {
        case 1:

            break;

        case 2:
            printf("enter speed and angle: ");
            cin >> speed >> angle;
            angle = angle / 180 * PI;
            WHEEL::move_front(speed, angle);
            break;

        case 3:
            printf("enter picture code: ");
            cin >> sign_number;
            VISION::takingPhoto(sign_number);
            cout << VISION::isDetected << endl;
            break;

        case 4:
            printf("enter picture code: ");
            cin >> sign_number;
            runAndDetectImage(sign_number);
            break;

        default:
            break;
        }
    }
    return 0;
}
