#include "race/turn_left_right.h"

void turnScript(void)
{
    ROS_INFO("\n==TURN SCRIPT==\n");
    int testCounter = 0;
    WHEEL::moveStraightLine(1); // straight forward
    WHEEL::move_front(0, 1.57); // turn left
    do
    {
        WHEEL::move_front(3, 0);
        VISION::takingPhoto(3);
        // std::cout << VISION::direction << " " << VISION::isDetected << endl;
    } while (!VISION::isDetected);
    if (VISION::direction == 'L')
        WHEEL::move_front(0, 1.57); // turn left
    else if (VISION::direction == 'R')
        WHEEL::move_front(0, -1.57); // turn left

    return;
}
