#include "race/turn_left_right.h"

void turnScript(void)
{
    ROS_INFO("\n==TURN SCRIPT==\n");
    double x;
    double y;
    double z;
    int testCounter = 0;
    WHEEL::move_front(3, 0); // straight forward
    WHEEL::move_front(0, 1.57); // turn left
    do
    {
        WHEEL::move_front(3, 0); // straight forward for a while
        VISION::takingPhoto(3);
        // std::cout << VISION::direction << " " << VISION::isDetected << endl;
    } while (!VISION::isDetected);
    VISION::isDetected = !VISION::isDetected;
    if (VISION::direction == "turn left")
        WHEEL::move_front(0, 1.57); // turn left
    else if (VISION::direction == "turn right")
        WHEEL::move_front(0, -1.57); // turn left

    return;
}
