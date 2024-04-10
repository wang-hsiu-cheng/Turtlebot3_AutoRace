#include "run.h"

void run1()
{
    Node pathConfig = LoadFile(road1_yaml);
    run(pathConfig);
}
void run2(char direction)
{
    if (direction == 'r')
        Node pathConfig = LoadFile(road2R_yaml);
    else if (direction == 'l')
        Node pathConfig = LoadFile(road2L_yaml);
    run(pathConfig);
}
void run5()
{
    Node pathConfig = LoadFile(road5_yaml);
    run(pathConfig);
}
void run6()
{
    Node pathConfig = LoadFile(road6_yaml);
    run(pathConfig);
}
void run(Node pathConfig)
{
    for (auto point : pathConfig)
    {
        distance = point["da"][0].as<double>();
        angle = point["da"][1].as<double>();
        angleRad = angle / 180 * PI;
        WHEEL::moveTo(distance, angle);
    }
}