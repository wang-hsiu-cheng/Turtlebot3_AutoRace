#include "race/detect.h"

void DETECT::init(ros::NodeHandle nh)
{
    vl53_subscriber = nh.subscribe("/distance",1,DETECT::callback);
}

void DETECT::callback(const std_msgs::Float32::ConstPtr &msg){
    vl53_sub.data = msg->data;
    ROS_INFO("distance: %f",vl53_sub.data);
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
    a = vl53_sub.data / 1000;
    loop_rate.sleep();
    ros::spinOnce();
    WHEEL::moveTo(0,0.52359877);
    ros::spinOnce;
    b = vl53_sub.data / 1000;;
    loop_rate.sleep();
    WHEEL::moveTo(0,1.04719753);
    ros::spinOnce;
    c = vl53_sub.data / 1000;;
    loop_rate.sleep();
    WHEEL::moveTo(0,-0.52359877);
    if (abs(0.1 - a) <= 0.03)
        return 'r';
    else if (abs(0.1 - b) <= 0.03)
        return 'l';
    else if (abs(a-b) > 0.1)
    {
        if(a < b)
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
    return 0;
}
int DETECT::fanceDetect()
{
    ros::Rate loop_rate(10);
    ros::spinOnce();
    float c = vl53_sub.data / 1000;
    float prev_c = 10000;
    if (c <= 0.05 && c >=0.1)
    {
        prev_c = c;
        return 1;
    }
        
    else{
        if(abs(prev_c - c)>=0.05)
        return 2;
    }  
}
void DETECT::positionCheck()
{
    return;
}
