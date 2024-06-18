#include "race/detect.h"

void DETECT::init(ros::NodeHandle nh)
{
    vl53_subscriber = nh.subscribe("/distance", 1, DETECT::callback);
}

void DETECT::callback(const std_msgs::Float32::ConstPtr &msg)
{
    vl53_sub.data = msg->data;
    ROS_INFO("distance: %f", vl53_sub.data);
}
void DETECT::runAndDetectImage(const int sign_number)
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
char DETECT::turnSignDetect(void)
{
    ros::Rate loop_rate(10);
    float a = 10000, b = 10000, c = 10000;
    ros::spinOnce();
    a = vl53_sub.data / 1000;
    WHEEL::moveTo(0, 0.52359877);
    loop_rate.sleep();
    ros::spinOnce();
    b = vl53_sub.data / 1000;
    WHEEL::moveTo(0, -1.04719753);
    loop_rate.sleep();
    ros::spinOnce();
    c = vl53_sub.data / 1000;
    WHEEL::moveTo(0, 0.52359877);
    loop_rate.sleep();
    if (abs(0.4 - a) <= 0.03)
        return 'r';
    else if (abs(0.4 - b) <= 0.1)
        return 'l';
    else if (abs(a - b) > 0.1)
    {
        if (a < b)
            return 'r';
        else
            return 'l';
    }
    else
    {
        if (a < c && b > c)
            return 'r';
        else if (a > c && b < c)
            return 'l';
        else
            return 'r';
    }
}
int DETECT::fanceDetect()
{
    ros::Rate loop_rate(10);
    float c = 0;
    float prev_c = 0;
    int i = 0;
    if (!ros::ok())
        return 0;
    do
    {
        ros::spinOnce();
        prev_c = c;
        c = vl53_sub.data / 1000;
    } while (c < 0.05 || c > 0.2); // fance haven't fall down
    // face down
    do
    {
        ros::spinOnce();
        prev_c = c;
        c = vl53_sub.data / 1000;
    } while (abs(prev_c - c) < 0.05 || c <= 0.2);
    while (i < 100)
    {
        ros::spinOnce();
        c = vl53_sub.data / 1000;
        if (c > 0.2)
            i++;
    }
    // fance rise (prev_c - c large & c > 0.1)
    return 1;
}
void DETECT::positionCheck()
{
    return;
}
