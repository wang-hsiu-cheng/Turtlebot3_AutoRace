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
        distance = point["da"][1].as<double>();
        angle = point["da"][2].as<double>();
        angleRad = angle / 180 * PI;
        if (point["da"][0].as<int>() == 1)
            WHEEL::moveTo(distance, angleRad);
        else if (point["da"][0].as<int>() == 0)
            WHEEL::moveCurve(distance, angleRad);
        else
            return;
    }
}