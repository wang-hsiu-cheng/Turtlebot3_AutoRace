#include "race/turn_left_right.h"

void turnScript(void)
{
    ROS_INFO("\n==TURN SCRIPT==\n");
    double x;
    double y;
    double z;
    WHEEL::moveTo(x, y, z); // straight forward
    WHEEL::moveTo(x, y, z); // turn left
    do
    {
        WHEEL::moveTo(x, y, z); // straight forward for a while
        VISION::takingPhoto(3);
    } while (!VISION::isDetected);
    VISION::isDetected = !VISION::isDetected;

    return;
}
