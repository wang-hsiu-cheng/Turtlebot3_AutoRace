#include "race/run.h"

void run1()
{
    printf("st1\n");
    Node pathConfig = LoadFile(road1_yaml);
    run(pathConfig);
}
void run2(char direction)
{
    if (direction == 'r')
    {
        Node pathConfig = LoadFile(road2R_yaml);
        run(pathConfig);
    }
    else if (direction == 'l')
    {
        Node pathConfig = LoadFile(road2L_yaml);
        run(pathConfig);
    }
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
    printf("st\n");
    for (auto point : pathConfig)
    {
        distance = point["da"][0].as<double>();
        angle = point["da"][1].as<double>();
        angleRad = angle / 180 * PI;
        WHEEL::moveTo(distance, angleRad);
        // WHEEL::move_front(0, angleRad);
        // WHEEL::moveStraightLine(distance);
    }
}