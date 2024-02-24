#include "race/turn_left_right.h"

void turnScript(void)
{
    ROS_INFO("\n==TURN SCRIPT==\n");
    double x;
    double y;
    double z;
    WHEEL::moveTo(x, y, z);
    WHEEL::moveTo(x, y, z);
    do
    {
        WHEEL::moveTo(x, y, z);
        VISION::takingPhoto(3);
    } while (!VISION::isDetected);
}
