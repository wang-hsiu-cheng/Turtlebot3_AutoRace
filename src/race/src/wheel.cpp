#include <math.h>
#include "race/wheel.h"
// ******miffy edited this@0229. miffy dont know anything.
// declare NodeHandle and pub/sub
void WHEEL::init(ros::NodeHandle nh)
{
    // wheel_publisher = nh.advertise<geometry_msgs::Point>("wheel_toSTM", 1);
    wheel_subscriber = nh.subscribe("/cmd_vel", 1, WHEEL::callback);
    wheel_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10); // miffy use newtopic to the base controller.
    nh.getParam("calibration_x_intercept", calibration_x_intercept);
    nh.getParam("calibration_y_intercept", calibration_y_intercept);
    nh.getParam("calibration_z_intercept", calibration_z_intercept);
    nh.getParam("calibration_x", calibration_x);
    nh.getParam("calibration_y", calibration_y);
    nh.getParam("calibration_z", calibration_z);

    nh.getParam("max_xy", max_xy);
    nh.getParam("min_xy", min_xy);
    nh.getParam("max_z", max_z);
    nh.getParam("acc_xy", acc_xy);
    nh.getParam("acc_zz", acc_zz);
    nh.getParam("maxUP_xy", maxUP_xy);
    nh.getParam("accUP_xy", accUP_xy);

    nh.getParam("kp", kp);
    nh.getParam("fod_xy", fod_xy);
    nh.getParam("fod_z", fod_z);
    nh.getParam("kp_xy", kp_xy);
    nh.getParam("kp_z", kp_z);

    nh.getParam("x_tol_margin", x_tol_margin);
    nh.getParam("y_tol_margin", y_tol_margin);
    nh.getParam("z_tol_margin", z_tol_margin);
}

// [Past way]encdoer callback function and publish
// void WHEEL::callback(const geometry_msgs::Twist::ConstPtr &vel)
// {
//     wheel_sub.x = vel->x;
//     wheel_sub.y = vel->y;
//     wheel_sub.z = vel->z;

//     data_check = true;
// }

void WHEEL::callback(const geometry_msgs::Twist::ConstPtr &msg) // miffy changed the message type to Twist
{
    // ROS_INFO("speed report: x= %f,y= %f,theta= %f",msg->linear.x,msg->linear.y,msg->angular.z);
    wheel_sub.linear.x = msg->linear.x;
    wheel_sub.linear.y = msg->linear.y;
    wheel_sub.linear.z = msg->angular.z;
}

float min(float a, float b)
{
    if (a < b)
        return a;
    else
        return b;
}
float max(double a, double b)
{
    if (a > b)
        return a;
    else
        return b;
}
// void WHEEL::move(vector<double> point_left, vector<double> vector_left, vector<double> point_right, vector<double> vector_right)
// {
//     // determine where should wheel move to.
// }
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
    float angle_now = 0;
    float angle_err = 1;
    float angle_const;
    ros::Rate loop_rate(10); // 设置发布频率为10Hz
    ros::Time lastTime = ros::Time::now();
    ros::Time currentTime;
    double dt;

    while (abs(angle_err) > 0.01 && ros::ok())
    {
        if (angle_err <= 0.1745329252)
            angle_const = 0.087266462599716; // 5degree per second
        else if (angle_err <= 0.5235988)
            angle_const = 0.174532925199433; // 10degree per second
        else if (angle_err <= 1.0471976)
            angle_const = 0.349065850398866; // 20degree per second
        else if (angle_err <= 1.57079632679)
            angle_const = 0.523598775598299; // 30degree per second

        if ((abs(angle_err) / angle_err) < 0)
            angle_const = -angle_const;
        wheel_pub.linear.x = 0;
        wheel_pub.linear.y = 0;
        wheel_pub.angular.z = angle_const;
        printf("ang rad: %.3f, ang err: %.3f, ang const: %.3f, ang now: %.3f\n", angle_rad, angle_err, angle_const, angle_now);
        wheel_publisher.publish(wheel_pub);
        currentTime = ros::Time::now();
        dt = (currentTime - lastTime).toSec();
        lastTime = currentTime;
        angle_now += angle_const * dt;
        angle_err = angle_rad - angle_now;
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
    // float speed_x = whole_speed*cos(angle_rad);
    // float speed_y = whole_speed*sin(angle_rad);
    // wheel_pub.linear.x = whole_speed;
    // // wheel_pub.linear.y = speed_y;
    // wheel_pub.angular.z = angle_rad;
    // // wheel_publisher.publish(wheel_pub);
    // ros::Rate loop_rate(10); // 设置发布频率为10Hz
    // int testCounter = 0;
    // if (ros::ok() )
    // {
    //     testCounter ++;
    //     std::cout << testCounter << std::endl;
    //     wheel_publisher.publish(wheel_pub);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
}
int WHEEL::moveStraightLine(float distance)
{
    float xDeltaMove = 0;
    float xDistance = 1;
    float xVelocityNow, xVelocityBefore;

    ros::Rate loop_rate(10); // 设置发布频率为10Hz
    ros::Time lastTime = ros::Time::now();
    ros::Time currentTime;
    double dt;

    while (abs(xDistance) > 0.03 && ros::ok())
    {
        ros::spinOnce();
        xVelocityBefore = wheel_sub.linear.x;
        currentTime = ros::Time::now();
        dt = (currentTime - lastTime).toSec();
        xDeltaMove += xVelocityBefore * dt;
        xDistance = distance - xDeltaMove;

        if (xDistance <= 0.05)
            xVelocityNow = 0.03;
        else if (xDistance <= 0.1)
            xVelocityNow = 0.1;
        else if (xDistance <= 0.2)
            xVelocityNow = 0.2;
        else if (xDistance <= 0.5)
            xVelocityNow = 0.5;
        else
            xVelocityNow = 1;

        // if ((abs(x_vel_err) / x_vel_err) < 0)
        //     x_vel_const = -x_vel_const;
        wheel_pub.linear.x = xVelocityNow;
        printf("distance: %.3f, pos err: %.3f, vel now: %.3f, vel before: %.3f\n", distance, xDistance, xVelocityNow, xVelocityBefore);
        wheel_publisher.publish(wheel_pub);
        lastTime = currentTime;
        loop_rate.sleep();
    }
    wheel_pub.linear.x = 0;
    wheel_publisher.publish(wheel_pub);
    loop_rate.sleep();
    return 1; // when reach the goal velo will return 1
}
int WHEEL::stop()
{

    double accerlearation = 0.01;
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
void WHEEL::moveTo(double x_cor, double y_cor, double z_cor)
{
    /* Clibration Coefficient */
    if (x_cor != 0)
        x_cor += calibration_x_intercept;
    if (y_cor != 0)
        y_cor += calibration_y_intercept;
    if (z_cor != 0)
        z_cor += calibration_z_intercept;

    x_cor /= calibration_x;
    y_cor /= calibration_y;
    z_cor /= calibration_z;

    z_cor *= 3.1415926 / 180;
    double acc_x = 0, acc_y = 0, acc_z = 0;
    double x_now = 0, y_now = 0, z_now = 0;
    double x_err = x_cor, y_err = y_cor, z_err = z_cor; // distance between now & goal
    double time_now, time_before;
    double xVelocityBefore, y_vel_before, z_vel_before; // velocity of previous instance
    double pub_x = 0, pub_y = 0, pub_z = 0;
    bool flag = false; // flag for NOT integral on first instance

    while ((fabs(x_err) > x_tol_margin || fabs(y_err) > y_tol_margin || fabs(z_err) > z_tol_margin) && ros::ok())
    {
        // calculate error and pub new speed
        x_err = x_cor - x_now;
        y_err = y_cor - y_now;
        z_err = z_cor - z_now;

        /* velocity profile */
        pub_x = 0;
        pub_y = 0;
        pub_z = 0;

        /// accerlation ///
        if (fabs(x_err) > fabs(fod_xy * x_cor) && fabs(x_err) > x_tol_margin)
        {
            pub_x = acc_x;
            acc_x += (x_err > 0) ? acc_xy : -acc_xy;
            if (acc_x >= max_xy)
                pub_x = max_xy;
            if (acc_x <= -max_xy)
                pub_x = -max_xy;
        }

        if (fabs(y_err) > fabs(fod_xy * y_cor) && fabs(y_err) > y_tol_margin)
        {
            pub_y = acc_y;
            acc_y += (y_err > 0) ? acc_xy : -acc_xy;
            if (acc_y >= max_xy)
                pub_y = max_xy;
            if (acc_y <= -max_xy)
                pub_y = -max_xy;
        }

        if (fabs(z_err) > fabs(fod_z * z_cor) && fabs(z_err) > z_tol_margin)
        {
            pub_z = acc_z;
            acc_z += (z_err > 0) ? acc_zz : -acc_zz;
            if (acc_z >= max_z)
                pub_z = max_z;
            if (acc_z <= -max_z)
                pub_z = -max_z;
        }

        /// deceleration ///
        if (fabs(x_err) <= fabs(fod_xy * x_cor) && x_cor != 0)
        {
            pub_x = kp_xy * x_err;
            if (pub_x >= max_xy)
                pub_x = max_xy;
            if (pub_x <= -max_xy)
                pub_x = -max_xy;
            if (pub_x <= min_xy && pub_x > 0)
                pub_x = min_xy;
            if (pub_x >= -min_xy && pub_x < 0)
                pub_x = -min_xy;
        }
        if (fabs(y_err) <= fabs(fod_xy * y_cor) && y_cor != 0)
        {
            pub_y = kp_xy * y_err;
            if (pub_y >= max_xy)
                pub_y = max_xy;
            if (pub_y <= -max_xy)
                pub_y = -max_xy;
            if (pub_y <= min_xy && pub_y > 0)
                pub_y = min_xy;
            if (pub_y >= -min_xy && pub_y < 0)
                pub_y = -min_xy;
        }
        if (fabs(z_err) <= fabs(fod_z * z_cor) && z_cor != 0)
        {
            pub_z = kp_z * z_err;
            if (pub_z >= max_z)
                pub_z = max_z;
            if (pub_z <= -max_z)
                pub_z = -max_z;
        }

        wheel_pub.linear.x = pub_x;
        wheel_pub.linear.y = pub_y;
        wheel_pub.angular.z = pub_z;
        wheel_publisher.publish(wheel_pub);

        /* velocity profile */
        data_check = false;
        while (!data_check && ros::ok())
            ros::spinOnce();

        // integral (unit: cm/s)
        time_now = ros::Time::now().toSec();

        if (flag)
        {
            x_now += (time_now - time_before) * (wheel_sub.linear.x + xVelocityBefore) / 2;
            y_now += (time_now - time_before) * (wheel_sub.linear.y + y_vel_before) / 2;
            z_now += (time_now - time_before) * (wheel_sub.angular.z + z_vel_before) / 2;
        }
        flag = true;

        xVelocityBefore = wheel_sub.linear.x;
        y_vel_before = wheel_sub.linear.y;
        z_vel_before = wheel_sub.angular.z;
        time_before = time_now;
        ros::Duration(0.005).sleep();
    }

    // reaching goal and pub speed 0
    while ((wheel_sub.linear.x != 0 || wheel_sub.linear.y != 0 || wheel_sub.angular.z != 0) && ros::ok())
    {
        wheel_pub.linear.x = 0;
        wheel_pub.linear.y = 0;
        wheel_pub.angular.z = 0;
        wheel_publisher.publish(wheel_pub);

        data_check = false;
        while (!data_check && ros::ok())
            ros::spinOnce();
    }
}

void WHEEL::moveUP(double x_cor, double y_cor, double z_cor)
{
    /* Clibration Coefficient */
    if (x_cor != 0)
        x_cor += calibration_x_intercept;
    if (y_cor != 0)
        y_cor += calibration_y_intercept;
    if (z_cor != 0)
        z_cor += calibration_z_intercept;

    x_cor /= calibration_x;
    y_cor /= calibration_y;
    z_cor /= calibration_z;

    z_cor *= 3.1415926 / 180;
    double acc_x = 0, acc_y = 0, acc_z = 0;
    double x_now = 0, y_now = 0, z_now = 0;
    double x_err = x_cor, y_err = y_cor, z_err = z_cor; // distance between now & goal
    double time_now, time_before;
    double xVelocityBefore, y_vel_before, z_vel_before; // velocity of previous instance
    double pub_x = 0, pub_y = 0, pub_z = 0;
    bool flag = false; // flag for NOT integral on first instance

    while ((fabs(x_err) > x_tol_margin || fabs(y_err) > y_tol_margin || fabs(z_err) > z_tol_margin) && ros::ok())
    {
        // calculate error and pub new speed
        x_err = x_cor - x_now;
        y_err = y_cor - y_now;
        z_err = z_cor - z_now;

        /* velocity profile */
        pub_x = 0;
        pub_y = 0;
        pub_z = 0;

        /// accerlation ///
        if (fabs(x_err) > fabs(fod_xy * x_cor) && fabs(x_err) > x_tol_margin)
        {
            pub_x = acc_x;
            acc_x += (x_err > 0) ? accUP_xy : -accUP_xy;
            if (acc_x >= maxUP_xy)
                pub_x = maxUP_xy;
            if (acc_x <= -maxUP_xy)
                pub_x = -maxUP_xy;
        }

        if (fabs(y_err) > fabs(fod_xy * y_cor) && fabs(y_err) > y_tol_margin)
        {
            pub_y = acc_y;
            acc_y += (y_err > 0) ? accUP_xy : -accUP_xy;
            if (acc_y >= maxUP_xy)
                pub_y = maxUP_xy;
            if (acc_y <= -maxUP_xy)
                pub_y = -maxUP_xy;
        }

        if (fabs(z_err) > fabs(fod_z * z_cor) && fabs(z_err) > z_tol_margin)
        {
            pub_z = acc_z;
            acc_z += (z_err > 0) ? acc_zz : -acc_zz;
            if (acc_z >= max_z)
                pub_z = max_z;
            if (acc_z <= -max_z)
                pub_z = -max_z;
        }

        /// deceleration ///
        if (fabs(x_err) <= fabs(fod_xy * x_cor) && x_cor != 0)
        {
            pub_x = kp_xy * x_err;
            if (pub_x >= maxUP_xy)
                pub_x = maxUP_xy;
            if (pub_x <= -maxUP_xy)
                pub_x = -maxUP_xy;
            if (pub_x <= min_xy && pub_x > 0)
                pub_x = min_xy;
            if (pub_x >= -min_xy && pub_x < 0)
                pub_x = -min_xy;
        }
        if (fabs(y_err) <= fabs(fod_xy * y_cor) && y_cor != 0)
        {
            pub_y = kp_xy * y_err;
            if (pub_y >= maxUP_xy)
                pub_y = maxUP_xy;
            if (pub_y <= -maxUP_xy)
                pub_y = -maxUP_xy;
            if (pub_y <= min_xy && pub_y > 0)
                pub_y = min_xy;
            if (pub_y >= -min_xy && pub_y < 0)
                pub_y = -min_xy;
        }
        if (fabs(z_err) <= fabs(fod_z * z_cor) && z_cor != 0)
        {
            pub_z = kp_z * z_err;
            if (pub_z >= max_z)
                pub_z = max_z;
            if (pub_z <= -max_z)
                pub_z = -max_z;
        }

        wheel_pub.linear.x = pub_x;
        wheel_pub.linear.y = pub_y;
        wheel_pub.angular.z = pub_z;
        wheel_publisher.publish(wheel_pub);

        /* velocity profile */
        data_check = false;
        while (!data_check && ros::ok())
            ros::spinOnce();

        // integral (unit: cm/s)
        time_now = ros::Time::now().toSec();

        if (flag)
        {
            x_now += (time_now - time_before) * (wheel_sub.linear.x + xVelocityBefore) / 2;
            y_now += (time_now - time_before) * (wheel_sub.linear.y + y_vel_before) / 2;
            z_now += (time_now - time_before) * (wheel_sub.angular.z + z_vel_before) / 2;
        }
        flag = true;

        xVelocityBefore = wheel_sub.linear.x;
        y_vel_before = wheel_sub.linear.y;
        z_vel_before = wheel_sub.angular.z;
        time_before = time_now;
        ros::Duration(0.005).sleep();
    }

    // reaching goal and pub speed 0
    while ((wheel_sub.linear.x != 0 || wheel_sub.linear.y != 0 || wheel_sub.angular.z != 0) && ros::ok())
    {
        wheel_pub.linear.x = 0;
        wheel_pub.linear.y = 0;
        wheel_pub.angular.z = 0;
        wheel_publisher.publish(wheel_pub);

        data_check = false;
        while (!data_check && ros::ok())
            ros::spinOnce();
    }
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
