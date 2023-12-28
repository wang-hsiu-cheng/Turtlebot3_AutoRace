#include "race/mecanum.h"


// declare NodeHandle and pub/sub
void MECANUM::init(ros::NodeHandle nh){
    mecanum_publisher = nh.advertise<geometry_msgs::Point>("mecanum_toSTM", 1);
    mecanum_subscriber = nh.subscribe("mecanum_fromSTM", 1, MECANUM::callback);

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

// encdoer callback function and publish
void MECANUM::callback(const geometry_msgs::Point::ConstPtr &vel){
    mecanum_sub.x = vel->x;
    mecanum_sub.y = vel->y;
    mecanum_sub.z = vel->z;

    data_check = true;
}

void MECANUM::moveTo(double x_cor, double y_cor, double z_cor){
/* Clibration Coefficient */
    if(x_cor!=0) x_cor += calibration_x_intercept;
    if(y_cor!=0) y_cor += calibration_y_intercept;
    if(z_cor!=0) z_cor += calibration_z_intercept;

    x_cor /= calibration_x;
    y_cor /= calibration_y;
    z_cor /= calibration_z;

    z_cor *= 3.1415926 / 180;
    double acc_x = 0, acc_y = 0, acc_z = 0;
    double x_now = 0, y_now = 0, z_now = 0;
    double x_err = x_cor, y_err = y_cor, z_err = z_cor; // distance between now & goal
    double time_now, time_before;
    double x_vel_before, y_vel_before, z_vel_before; // velocity of previous instance
    double pub_x=0, pub_y=0, pub_z=0;
    bool flag = false;                               // flag for NOT integral on first instance

    while ((fabs(x_err) > x_tol_margin || fabs(y_err) > y_tol_margin || fabs(z_err) > z_tol_margin) && ros::ok()){
        // calculate error and pub new speed
        x_err = x_cor - x_now;
        y_err = y_cor - y_now;
        z_err = z_cor - z_now;

        /* velocity profile */
        pub_x = 0;
        pub_y = 0;
        pub_z = 0;

        /// accerlation ///
        if (fabs(x_err) > fabs(fod_xy * x_cor) && fabs(x_err) > x_tol_margin){
            pub_x = acc_x;
            acc_x += (x_err > 0) ? acc_xy : -acc_xy;
            if (acc_x >= max_xy) pub_x = max_xy;
            if (acc_x <= -max_xy) pub_x = -max_xy;
        }

        if (fabs(y_err) > fabs(fod_xy * y_cor) && fabs(y_err) > y_tol_margin){
            pub_y = acc_y;
            acc_y += (y_err > 0) ? acc_xy : -acc_xy;
            if (acc_y >= max_xy) pub_y = max_xy;
            if (acc_y <= -max_xy) pub_y = -max_xy;
        }

        if (fabs(z_err) > fabs(fod_z *  z_cor) && fabs(z_err) > z_tol_margin){
            pub_z = acc_z;
            acc_z += (z_err > 0) ? acc_zz : -acc_zz; 
            if (acc_z >= max_z) pub_z = max_z;
            if (acc_z <= -max_z) pub_z = -max_z;
        }

        /// deceleration ///
        if (fabs(x_err) <= fabs(fod_xy * x_cor) && x_cor != 0){
            pub_x = kp_xy * x_err;
            if (pub_x >= max_xy) pub_x = max_xy;
            if (pub_x <= -max_xy) pub_x = -max_xy;
            if (pub_x <= min_xy && pub_x > 0) pub_x = min_xy;
            if (pub_x >= -min_xy && pub_x < 0) pub_x = -min_xy;
        }
        if (fabs(y_err) <= fabs(fod_xy * y_cor) && y_cor != 0){
            pub_y = kp_xy * y_err;
            if (pub_y >= max_xy) pub_y = max_xy;
            if (pub_y <= -max_xy) pub_y = -max_xy;
            if (pub_y <= min_xy && pub_y > 0) pub_y = min_xy;
            if (pub_y >= -min_xy && pub_y < 0) pub_y = -min_xy;
        }
        if (fabs(z_err) <= fabs(fod_z * z_cor) && z_cor != 0){
            pub_z = kp_z * z_err; 
            if (pub_z >= max_z) pub_z = max_z;
            if (pub_z <= -max_z) pub_z = -max_z;
        }

        mecanum_pub.x = pub_x; 
        mecanum_pub.y = pub_y; 
        mecanum_pub.z = pub_z; 
        mecanum_publisher.publish(mecanum_pub);


        /* velocity profile */
        data_check = false;
        while (!data_check && ros::ok() ) ros::spinOnce();

        // integral (unit: cm/s)
        time_now = ros::Time::now().toSec();

        if (flag){
            x_now += (time_now - time_before) * (mecanum_sub.x + x_vel_before) / 2;
            y_now += (time_now - time_before) * (mecanum_sub.y + y_vel_before) / 2;
            z_now += (time_now - time_before) * (mecanum_sub.z + z_vel_before) / 2;
        }
        flag = true;

        x_vel_before = mecanum_sub.x;
        y_vel_before = mecanum_sub.y;
        z_vel_before = mecanum_sub.z;
        time_before = time_now;
        ros::Duration(0.005).sleep();
    }

    // reaching goal and pub speed 0
    while ((mecanum_sub.x != 0 || mecanum_sub.y != 0 || mecanum_sub.z != 0) && ros::ok()){
        mecanum_pub.x = 0;
        mecanum_pub.y = 0;
        mecanum_pub.z = 0;
        mecanum_publisher.publish(mecanum_pub);

        data_check = false;
        while (!data_check && ros::ok()) ros::spinOnce();
    }
}

void MECANUM::moveUP(double x_cor, double y_cor, double z_cor){
/* Clibration Coefficient */
    if(x_cor!=0) x_cor += calibration_x_intercept;
    if(y_cor!=0) y_cor += calibration_y_intercept;
    if(z_cor!=0) z_cor += calibration_z_intercept;

    x_cor /= calibration_x;
    y_cor /= calibration_y;
    z_cor /= calibration_z;

    z_cor *= 3.1415926 / 180;
    double acc_x = 0, acc_y = 0, acc_z = 0;
    double x_now = 0, y_now = 0, z_now = 0;
    double x_err = x_cor, y_err = y_cor, z_err = z_cor; // distance between now & goal
    double time_now, time_before;
    double x_vel_before, y_vel_before, z_vel_before; // velocity of previous instance
    double pub_x=0, pub_y=0, pub_z=0;
    bool flag = false;                               // flag for NOT integral on first instance

    while ((fabs(x_err) > x_tol_margin || fabs(y_err) > y_tol_margin || fabs(z_err) > z_tol_margin) && ros::ok()){
        // calculate error and pub new speed
        x_err = x_cor - x_now;
        y_err = y_cor - y_now;
        z_err = z_cor - z_now;

        /* velocity profile */
        pub_x = 0;
        pub_y = 0;
        pub_z = 0;

        /// accerlation ///
        if (fabs(x_err) > fabs(fod_xy * x_cor) && fabs(x_err) > x_tol_margin){
            pub_x = acc_x;
            acc_x += (x_err > 0) ? accUP_xy : -accUP_xy;
            if (acc_x >= maxUP_xy) pub_x = maxUP_xy;
            if (acc_x <= -maxUP_xy) pub_x = -maxUP_xy;
        }

        if (fabs(y_err) > fabs(fod_xy * y_cor) && fabs(y_err) > y_tol_margin){
            pub_y = acc_y;
            acc_y += (y_err > 0) ? accUP_xy : -accUP_xy;
            if (acc_y >= maxUP_xy) pub_y = maxUP_xy;
            if (acc_y <= -maxUP_xy) pub_y = -maxUP_xy;
        }

        if (fabs(z_err) > fabs(fod_z *  z_cor) && fabs(z_err) > z_tol_margin){
            pub_z = acc_z;
            acc_z += (z_err > 0) ? acc_zz : -acc_zz; 
            if (acc_z >= max_z) pub_z = max_z;
            if (acc_z <= -max_z) pub_z = -max_z;
        }

        /// deceleration ///
        if (fabs(x_err) <= fabs(fod_xy * x_cor) && x_cor != 0){
            pub_x = kp_xy * x_err;
            if (pub_x >= maxUP_xy) pub_x = maxUP_xy;
            if (pub_x <= -maxUP_xy) pub_x = -maxUP_xy;
            if (pub_x <= min_xy && pub_x > 0) pub_x = min_xy;
            if (pub_x >= -min_xy && pub_x < 0) pub_x = -min_xy;
        }
        if (fabs(y_err) <= fabs(fod_xy * y_cor) && y_cor != 0){
            pub_y = kp_xy * y_err;
            if (pub_y >= maxUP_xy) pub_y = maxUP_xy;
            if (pub_y <= -maxUP_xy) pub_y = -maxUP_xy;
            if (pub_y <= min_xy && pub_y > 0) pub_y = min_xy;
            if (pub_y >= -min_xy && pub_y < 0) pub_y = -min_xy;
        }
        if (fabs(z_err) <= fabs(fod_z * z_cor) && z_cor != 0){
            pub_z = kp_z * z_err; 
            if (pub_z >= max_z) pub_z = max_z;
            if (pub_z <= -max_z) pub_z = -max_z;
        }

        mecanum_pub.x = pub_x; 
        mecanum_pub.y = pub_y; 
        mecanum_pub.z = pub_z; 
        mecanum_publisher.publish(mecanum_pub);


        /* velocity profile */
        data_check = false;
        while (!data_check && ros::ok()) ros::spinOnce();

        // integral (unit: cm/s)
        time_now = ros::Time::now().toSec();

        if (flag){
            x_now += (time_now - time_before) * (mecanum_sub.x + x_vel_before) / 2;
            y_now += (time_now - time_before) * (mecanum_sub.y + y_vel_before) / 2;
            z_now += (time_now - time_before) * (mecanum_sub.z + z_vel_before) / 2;
        }
        flag = true;

        x_vel_before = mecanum_sub.x;
        y_vel_before = mecanum_sub.y;
        z_vel_before = mecanum_sub.z;
        time_before = time_now;
        ros::Duration(0.005).sleep();
    }

    // reaching goal and pub speed 0
    while ((mecanum_sub.x != 0 || mecanum_sub.y != 0 || mecanum_sub.z != 0) && ros::ok()){
        mecanum_pub.x = 0;
        mecanum_pub.y = 0;
        mecanum_pub.z = 0;
        mecanum_publisher.publish(mecanum_pub);

        data_check = false;
        while (!data_check && ros::ok()) ros::spinOnce();
    }
}

void MECANUM::readPath(std::string yaml_path){
    YAML::Node pathConfig = YAML::LoadFile(yaml_path);

    double x,y,z;
    for(auto xyz : pathConfig){
        x = xyz["xyz"][0].as<double>();
        y = xyz["xyz"][1].as<double>();
        z = xyz["xyz"][2].as<double>();

        MECANUM::moveTo(x,y,z);
    }
}
