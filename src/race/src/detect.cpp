#include "race/detect.h"

void road_line_detect(int sign_number)
{
    double x, y, z;
    VISION::taking_photo(0);
    WHEEL::moveTo(x, y, z);
    switch (sign_number)
    {
    case 1:
        VISION::taking_photo(2); // warning_sign_image
        break;
    case 2:
        VISION::taking_photo(4); // parking_sign_image
        break;
    case 3:
        VISION::taking_photo(5); // stop_sign_image
        break;
    case 4:
        VISION::taking_photo(7); // tunnel_sign_image
        break;
    default:
        break;
    }
    if (VISION::isDetected)
    {
        WHEEL::stop();
        VISION::isDetected = false;
        return;
    }
}