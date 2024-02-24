#include "race/run.h"

enum ImageName
{
    nothing,
    trafficLight,
    warningSign,
    turnSign,
    parkingSign,
    stopSign,
    fance,
    tunnelSign
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run");
    ros::NodeHandle nh;
    // ros::Publisher visionPub = nh.advertise<std_msgs::Int64>("camera", 10);
    // ros::Subscriber visionSub = nh.subscribe("run_fromCamera", 1, callback);

    nh.getParam("reset_state", reset_state);
    ROS_INFO("State Now: %d", reset_state);

    level = init_all_sensors(nh);

    if (reset_state == 0)
    {
        while (nh.ok())
            ros::spinOnce();
    }
    else
    {
        race_levels(reset_state);
    }

    return EXIT_SUCCESS;
}
// void Callback(const std_msgs::Int64::ConstPtr &msg)
// {
//     std::cout << "msg.data = " << msg->data << "\n";
//     data_check = msg->data;
// }

void race_levels(const int state)
{
    // while (ros::ok() && !data_check)
    // {
    //     visionPub.publish(1);
    //     ros::spinOnce();
    // }
    // data_check = !data_check;
    do
    {
        VISION::takingPhoto((int)trafficLight); // green_light_image
    } while (!VISION::isDetected);
    VISION::isDetected = !VISION::isDetected;
    runAndDetectImage((int)warningSign);

    level++;
    if (level >= state)
        return;

    turnScript();
    runAndDetectImage((int)warningSign);

    level++;
    if (level >= state)
        return;

    // avoid_wall_script();
    runAndDetectImage((int)parkingSign);

    level++;
    if (level >= state)
        return;

    // parking_script();
    runAndDetectImage((int)stopSign);

    level++;
    if (level >= state)
        return;

    // while (ros::ok() && !data_check)
    // {
    //     visionPub.publish(6);
    //     ros::spinOnce();
    // }
    // data_check = !data_check;
    do
    {
        VISION::takingPhoto((int)fance); // fance_image
    } while (!VISION::isDetected);
    VISION::isDetected = !VISION::isDetected;
    runAndDetectImage((int)tunnelSign);

    level++;
    if (level >= state)
        return;

    // navigation_system();
    runAndDetectImage((int)nothing);

    level++;
    if (level >= state)
        return;
}

int init_all_sensors(ros::NodeHandle nh)
{
    // WHEEL::init(nh);
    // IMU::init();
    return 1;
}
