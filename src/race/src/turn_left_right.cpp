#include "race/turn_left_right.h"

void turn_script(void)
{
    ROS_INFO("\n==TURN SCRIPT==\n");
    double x;
    double y;
    double z;
    WHEEL::moveTo(x, y, z);
    WHEEL::turn('l');
    do
    {
        WHEEL::moveTo(x, y, z);
        VISION::turn_sign_image();
    } while (!VISION::is_sign_exist)
}
