#include "race/detect.h"

void runAndDetectImage(const int sign_number)
{
    double x, y, z;

    do
    {
        CAMERA1::detectRoad();
        CAMERA1::isDetected = !CAMERA1::isDetected;
        WHEEL::moveTo(x, y, z);

        VISION::takingPhoto(sign_number);
    } while (!VISION::isDetected);

    VISION::isDetected = !VISION::isDetected;
    return;
}