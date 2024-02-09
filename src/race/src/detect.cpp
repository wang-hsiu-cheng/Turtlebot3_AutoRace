#include "race/detect.h"

void road_line_detect(int sign_number)
{
    VISION::road_line_image();
    // WHEEL::move(VISION::point_left, VISION::vector_left, VISION::point_right, VISION::vector_right);
    switch (sign_number)
    {
    case 1:
        VISION::warning_sign_image();
        break;
    case 2:
        VISION::parking_sign_image();
        break;
    case 3:
        VISION::stop_sign_image();
        break;
    case 4:
        VISION::tunnel_sign_image();
        break;
    default:
        break;
    }
    if (VISION::is_sign_exist)
    {
        WHEEL::stop();
        VISION::is_sign_exist = false;
        return;
    }
}