#include "race/detect.h"

void runAndDetectImage(const int sign_number)
{
    double x, y, z;
    int testCounter = 0;
    int loopTime = 20;
    int loopTiemCounter = 0;
    int error = 0;
    int errorThreshold = 20;
    double slop = 0;

    do
    {
        testCounter++;
        if (loopTiemCounter == loopTime)
        {
            CAMERA1::detectRoad();
            VISION::takingPhoto(sign_number);
            if (CAMERA1::isDetected)
                slop = CAMERA1::getSlop();
            else
                error++;
            loopTiemCounter = 0;
        }
        loopTiemCounter++;
        WHEEL::move_front(3, slop);
        if (error == errorThreshold)
            WHEEL::stop();
    } while (!VISION::isDetected && testCounter < 20);
    return;
}