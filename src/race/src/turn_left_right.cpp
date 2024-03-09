#include "race/turn_left_right.h"

void turnScript(void)
{
    ROS_INFO("\n==TURN SCRIPT==\n");
    double x;
    double y;
    double z;
    WHEEL::move_front(3, 0); // straight forward
    WHEEL::move_front(0, 1.57); // turn left
    do
    {
        WHEEL::move_front(3, 0); // straight forward for a while
        VISION::takingPhoto(3);
    } while (!VISION::isDetected);
    VISION::isDetected = !VISION::isDetected;

    return;
}
