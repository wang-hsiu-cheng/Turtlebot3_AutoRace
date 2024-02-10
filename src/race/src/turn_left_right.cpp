#include "race/turn_left_right.h"

void turn_script(void)
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
        VISION::taking_photo(3);
    } while (!VISION::isDetected);
}
