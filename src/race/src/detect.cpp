#include "race/detect.h"

void runAndDetectImage(const int sign_number)
{
    double x, y, z;
    int testCounter = 0;

    do
    {
        testCounter ++;
        CAMERA1::detectRoad();
        CAMERA1::isDetected = !CAMERA1::isDetected;
        WHEEL::move_front(3, 0);

        VISION::takingPhoto(sign_number);
    } while (!VISION::isDetected && testCounter < 20);

    VISION::isDetected = !VISION::isDetected;
    return;
}