#include <math.h>
#include "race/wheel.h"
// ******miffy edited this@0229. miffy dont know anything.
// declare NodeHandle and pub/sub
void WHEEL::init(ros::NodeHandle nh)
{
    // wheel_publisher = nh.advertise<geometry_msgs::Point>("wheel_toSTM", 1);
    wheel_subscriber = nh.subscribe("/cmd_vel", 1, WHEEL::callback);
    wheel_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10); // miffy use newtopic to the base controller.
    nh.getParam("rad_p_control_0", rad_p_control_0);
    nh.getParam("rad_p_control_1", rad_p_control_1);
    nh.getParam("rad_p_control_2", rad_p_control_2);
    nh.getParam("rad_p_control_3", rad_p_control_3);
    nh.getParam("omega_p_control_0", omega_p_control_0);
    nh.getParam("omega_p_control_1", omega_p_control_1);
    nh.getParam("omega_p_control_2", omega_p_control_2);
    nh.getParam("omega_p_control_3", omega_p_control_3);

    nh.getParam("distance_p_control_0", distance_p_control_0);
    nh.getParam("distance_p_control_1", distance_p_control_1);
    nh.getParam("distance_p_control_2", distance_p_control_2);
    nh.getParam("distance_p_control_3", distance_p_control_3);
    nh.getParam("velocity_p_control_0", velocity_p_control_0);
    nh.getParam("velocity_p_control_1", velocity_p_control_1);
    nh.getParam("velocity_p_control_2", velocity_p_control_2);
    nh.getParam("velocity_p_control_3", velocity_p_control_3);
    nh.getParam("velocity_p_control_4", velocity_p_control_4);
}

void WHEEL::callback(const geometry_msgs::Twist::ConstPtr &msg) // miffy changed the message type to Twist
{
    // ROS_INFO("speed report: x= %f,y= %f,theta= %f",msg->linear.x,msg->linear.y,msg->angular.z);
    wheel_sub.linear.x = msg->linear.x;
    wheel_sub.linear.y = msg->linear.y;
    wheel_sub.linear.z = msg->angular.z;
}

float min(double a, double b)
{
    return (a < b) ? a : b;
}
float max(double a, double b)
{
    return (a > b) ? a : b;
}
int WHEEL::move_front(int mode, float angle_rad)
{
    bool data_check, flag;
    // [Past reqirement] publish velocity and integrate the length that robot run.
    // if length is enough, then stop running and return.
    // [New Requriedment-20240226] receive the declination between robot & road edge
    float whole_speed; // set the max speed of forward dir.
    float xVelocityNow, y_vel_now, z_vel_now;
    float xVelocityBefore = 0, y_vel_before = 0, z_vel_before = 0;
    float acceleration = 0.01;
    /* do the rotation*/
    float angleNow = 0;
    float angleErr = 1;
    float angleConst;
    ros::Rate loop_rate(10); // 设置发布频率为10Hz
    ros::Time lastTime = ros::Time::now();
    ros::Time currentTime;
    double dt;

    while (abs(angleErr) > 0.01 && ros::ok())
    {
        if (angleErr <= 0.1745329252)
            angleConst = 0.087266462599716; // 5degree per second
        else if (angleErr <= 0.5235988)
            angleConst = 0.174532925199433; // 10degree per second
        else if (angleErr <= 1.0471976)
            angleConst = 0.349065850398866; // 20degree per second
        else if (angleErr <= 1.57079632679)
            angleConst = 0.523598775598299; // 30degree per second

        if ((abs(angleErr) / angleErr) < 0)
            angleConst = -angleConst;
        wheel_pub.linear.x = 0;
        wheel_pub.linear.y = 0;
        wheel_pub.angular.z = angleConst;
        printf("ang rad: %.3f, ang err: %.3f, ang const: %.3f, ang now: %.3f\n", angle_rad, angleErr, angleConst, angleNow);
        wheel_publisher.publish(wheel_pub);
        currentTime = ros::Time::now();
        dt = (currentTime - lastTime).toSec();
        lastTime = currentTime;
        angleNow += angleConst * dt;
        angleErr = angle_rad - angleNow;
        loop_rate.sleep();
    }
    wheel_pub.angular.z = 0;
    wheel_publisher.publish(wheel_pub);
    loop_rate.sleep();
    /* go foward after the rotation*/
    switch (mode) // mode stands for the speed: the larger the number, the faster the cat will run.
    {
    case 1:
        whole_speed = 1.5;
        break;
    case 2:
        whole_speed = 1;
        break;
    case 3:
        whole_speed = 0.5;
        break;
    case 4:
        whole_speed = 0;
        break;
    default:
        break;
    }
    /* velocity profile */
    data_check = false;
    flag = true;

    if (!data_check && ros::ok() && flag)
    {
        ros::spinOnce();
        xVelocityBefore = wheel_sub.linear.x;
        y_vel_before = wheel_sub.linear.y;
        z_vel_before = wheel_sub.angular.z;

        if (flag)
        {
            xVelocityNow = min((whole_speed), (xVelocityBefore + acceleration));
            // y_vel_now = min((whole_speed*sin(angle_rad)),(y_vel_before+acceleration));
            z_vel_now = 0;
        }
        flag = true;
        if (xVelocityNow >= whole_speed)
            flag = false;
        wheel_pub.linear.x = xVelocityNow;
        wheel_pub.linear.y = y_vel_now;
        wheel_pub.angular.z = z_vel_now;
        wheel_publisher.publish(wheel_pub);
        loop_rate.sleep();
    }
    if (!flag)
        return 1; // when reach the goal velo will return 1
    return 0;
}
int WHEEL::moveStraightLine(float distance)
{
    float xDeltaMove = 0;
    float remainDistance = 1;
    float xVelocityNow, xVelocityBefore, zVelocityBefore;

    ros::Rate loop_rate(10); // 设置发布频率为10Hz
    ros::Time lastTime = ros::Time::now();
    ros::Time currentTime;
    double dt;

    while (abs(remainDistance) > 0.001 && ros::ok())
    {
        ros::spinOnce();
        xVelocityBefore = wheel_sub.linear.x;
        zVelocityBefore = wheel_sub.angular.z;
        currentTime = ros::Time::now();
        dt = (currentTime - lastTime).toSec();
        xDeltaMove += xVelocityBefore * dt;
        remainDistance = distance - xDeltaMove;

        if (remainDistance <= 0.03)
            xVelocityNow = 0.1;
        else if (remainDistance <= 0.1)
            xVelocityNow = 0.35;
        else if (remainDistance <= 0.2)
            xVelocityNow = 0.5;
        else if (remainDistance <= 0.5)
            xVelocityNow = 0.7;
        else
            xVelocityNow = 0.8;

        wheel_pub.linear.x = xVelocityNow;
        wheel_pub.angular.z = zVelocityBefore;
        printf("distance: %.3f, pos err: %.3f, vel now: %.3f, vel before: %.3f, z angle: %.3f\n", distance, remainDistance, xVelocityNow, xVelocityBefore, zVelocityBefore);
        wheel_publisher.publish(wheel_pub);
        lastTime = currentTime;
        loop_rate.sleep();
    }
    wheel_pub.linear.x = 0;
    wheel_pub.angular.z = 0;
    wheel_publisher.publish(wheel_pub);
    loop_rate.sleep();
    return 1; // when reach the goal velo will return 1
}
void WHEEL::moveTo(int distance, double angleRad)
{
    double xVelocityNow, zVelocityNow;
    double xVelocityBefore = 0, zVelocityBefore = 0;
    double angleNow = 0;
    double angleErr = angleRad;
    double angleConst;
    double xDeltaMove = 0;
    double remainDistance = distance;
    ros::Rate loop_rate(10);
    ros::Time lastTime = ros::Time::now();
    ros::Time currentTime;
    double dt;

    while (abs(angleErr) > 0.01 && ros::ok())
    {
        if (angleErr <= rad_p_control_0)
            angleConst = omega_p_control_0; // 5degree per second
        else if (angleErr <= rad_p_control_1)
            angleConst = omega_p_control_1; // 10degree per second
        else if (angleErr <= rad_p_control_2)
            angleConst = omega_p_control_2; // 20degree per second
        else if (angleErr <= rad_p_control_3)
            angleConst = omega_p_control_3; // 30degree per second

        if ((abs(angleErr) / angleErr) < 0)
            angleConst = -angleConst;
        wheel_pub.linear.x = 0;
        wheel_pub.linear.y = 0;
        wheel_pub.angular.z = angleConst;
        printf("ang rad: %.3f, ang err: %.3f, ang const: %.3f, ang now: %.3f\n", angle_rad, angleErr, angleConst, angleNow);
        wheel_publisher.publish(wheel_pub);
        currentTime = ros::Time::now();
        dt = (currentTime - lastTime).toSec();
        lastTime = currentTime;
        angleNow += angleConst * dt;
        angleErr = angleRad - angleNow;
        loop_rate.sleep();
    }
    wheel_pub.angular.z = 0;
    wheel_publisher.publish(wheel_pub);
    loop_rate.sleep();

    while (abs(remainDistance) > 0.001 && ros::ok())
    {
        ros::spinOnce();
        xVelocityBefore = wheel_sub.linear.x;
        zVelocityBefore = wheel_sub.angular.z;
        currentTime = ros::Time::now();
        dt = (currentTime - lastTime).toSec();
        xDeltaMove += xVelocityBefore * dt;
        remainDistance -= xDeltaMove;

        if (remainDistance <= distance_p_control_0)
            xVelocityNow = velocity_p_control_0;
        else if (remainDistance <= distance_p_control_1)
            xVelocityNow = velocity_p_control_1;
        else if (remainDistance <= distance_p_control_2)
            xVelocityNow = velocity_p_control_2;
        else if (remainDistance <= distance_p_control_3)
            xVelocityNow = velocity_p_control_3;
        else
            xVelocityNow = velocity_p_control_4;

        wheel_pub.linear.x = xVelocityNow;
        wheel_pub.angular.z = zVelocityBefore;
        printf("distance: %.3f, pos err: %.3f, vel now: %.3f, vel before: %.3f, z angle: %.3f\n", distance, remainDistance, xVelocityNow, xVelocityBefore, zVelocityBefore);
        wheel_publisher.publish(wheel_pub);
        lastTime = currentTime;
        loop_rate.sleep();
    }
    wheel_pub.linear.x = 0;
    wheel_pub.angular.z = 0;
    wheel_publisher.publish(wheel_pub);
    loop_rate.sleep();
    return 1;
}
int WHEEL::stop()
{
    bool data_check, flag;
    float xVelocityNow, y_vel_now, z_vel_now;
    float xVelocityBefore = 0, y_vel_before = 0, z_vel_before = 0;
    data_check = false;
    int whole_speed = 0;

    while (!data_check && ros::ok())
    {
        ros::spinOnce();
        float acceleration = -0.01;
        xVelocityBefore = wheel_sub.linear.x;
        y_vel_before = wheel_sub.linear.y;
        z_vel_before = wheel_sub.angular.z;

        if (flag)
        {
            xVelocityNow = max(whole_speed, (xVelocityBefore + acceleration));
            y_vel_now = max(whole_speed, (y_vel_before + acceleration));
            z_vel_now = 0;
        }
        flag = true;
        wheel_pub.linear.x = xVelocityNow;
        wheel_pub.linear.y = y_vel_now;
        wheel_pub.angular.z = z_vel_now;
        wheel_publisher.publish(wheel_pub);
        if (xVelocityNow <= whole_speed && y_vel_now <= whole_speed)
            flag = false;
        ros::Duration(0.005).sleep();
    }
    if (!flag)
        return 1;
    return 0;
}

void WHEEL::readPath(std::string yaml_path)
{
    YAML::Node pathConfig = YAML::LoadFile(yaml_path);

    double x, y, z;
    for (auto xyz : pathConfig)
    {
        x = xyz["xyz"][0].as<double>();
        y = xyz["xyz"][1].as<double>();
        z = xyz["xyz"][2].as<double>();

        WHEEL::moveTo(x, y, z);
    }
}
