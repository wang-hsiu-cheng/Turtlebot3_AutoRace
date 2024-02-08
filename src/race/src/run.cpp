#include "race/run.h"

void init_all_sensors(ros::NodeHandle nh);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run");
    ros::NodeHandle nh;

    nh.getParam("reset_state", reset_state);
    ROS_INFO("State Now: %d", reset_state);

    level = init_all_sensors(nh);
    while (level < 9)
    {
        if (reset_state)
        {
            while (nh.ok())
                ros::spinOnce();
        }
        else
        {
            race_levels();
        }
    }
    return EXIT_SUCCESS;
}

void race_levels()
{
    // switch (level)
    // {
    // case 1:
    //     VISION::green_light_image();
    //     level++;
    //     break;
    // case 2:
    //     road_line_detect(1);
    //     level++;
    //     break;
    // case 3:
    //     turn_script();
    //     road_line_detect(1);
    //     level++;
    //     break;
    // case 4:
    //     avoid_wall_script();
    //     road_line_detect(2);
    //     level++;
    //     break;
    // case 5:
    //     parking_script();
    //     road_line_detect(3);
    //     level++;
    //     break;
    // case 6:
    //     VISION::fance_image();
    //     road_line_detect(4);
    //     level++;
    //     break;
    // case 7:
    //     navigation_system();
    //     level++;
    //     break;
    // case 8:
    //     road_line_detect(0);
    //     level++;
    //     break;
    // default:
    //     break;
    // }
    VISION::green_light_image();
    level++;
    road_line_detect(1);
    level++;
    turn_script();
    road_line_detect(1);
    level++;
    avoid_wall_script();
    road_line_detect(2);
    level++;
    parking_script();
    road_line_detect(3);
    level++;
    VISION::fance_image();
    road_line_detect(4);
    level++;
    navigation_system();
    road_line_detect(0);
    level++;
}

int init_all_sensors(ros::NodeHandle nh)
{
    MECANUM::init(nh);
    SCARA::init(nh);
    // IMU::init();
    return 1;
}

// #include "race/run.h"

// void init_all_sensors(ros::NodeHandle nh);

// int main(int argc, char **argv){
//     ros::init(argc, argv, "run");
//     ros::NodeHandle nh;

//     nh.getParam("reset_state", reset_state);
//     ROS_INFO("State Now: %d", reset_state);

//     init_all_sensors(nh);

//     switch(reset_state){
//         case 0:
//             while(nh.ok()) ros::spinOnce();

//         case 1:
//             run1(); run2(); run3();
//             break;

//         case 2:
//             run2(); run3();
//             break;

//         case 3:
//             run3();
//             break;
//     }
//     return EXIT_SUCCESS;
// }

// void init_all_sensors(ros::NodeHandle nh){
//     MECANUM::init(nh);
//     SCARA::init(nh);
//     // SWITCH::init();
//     // IMU::init();
// }
