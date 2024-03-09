#include "race/roadCamera.h"
#include "math.h"
#include "ros/ros.h"

void CAMERA1::detectRoad()
{
    VideoCapture cap(0); // 鏡頭編號依序從 012...
    Mat img;

    if (!cap.isOpened())
    { // 確認有連線到該編號的鏡頭
        cout << "Cannot open capture\n";
        return;
    }
    bool ret = cap.read(img);
    while (!ret)
    {
        cout << "Cant receive frame\n";
        ret = cap.read(img);
    }

    Mat originalImage = img.clone();
    Mat filtYellowImage = img.clone();
    Mat filtWhiteImage = img.clone();
    filtGraph(originalImage, filtYellowImage, 'y');
    filtGraph(originalImage, filtWhiteImage, 'w');
    roadLineImage(originalImage, filtYellowImage, filtWhiteImage);

    return;
}

void CAMERA1::filtGraph(Mat src, Mat &filteredImg, char color)
{
    Mat img_hsv, mask;
    cvtColor(img, img_hsv, COLOR_BGR2HSV);

    // CAMERA1::hue_m = 0;
    // CAMERA1::hue_M = 173;
    // CAMERA1::sat_m = 0;
    // CAMERA1::sat_M = 190;
    // CAMERA1::val_m = 191;
    // CAMERA1::val_M = 226;

    if (color == 'y')
    {
        // yellow range
        hue_m = 14;
        hue_M = 78;
        sat_m = 48;
        sat_M = 255;
        val_m = 108;
        val_M = 255;
    }
    else if (color == 'w')
    {
        // white range
        hue_m = 0;
        hue_M = 179;
        sat_m = 0;
        sat_M = 40;
        val_m = 175;
        val_M = 255;
    }

    Scalar lower(hue_m, sat_m, val_m);
    Scalar upper(hue_M, sat_M, val_M);
    inRange(img_hsv, lower, upper, mask);

    filteredImg = Mat::zeros(src.size(), CV_8UC3);
    bitwise_and(src, src, filteredImg, mask);

    return;
}

void CAMERA1::roadLineImage(Mat src, Mat &yellowImg, Mat &whiteImg)
{
    Mat thresh_yellow, thresh_white;

    cvtColor(yellowImg, yellowImg, COLOR_BGR2GRAY);
    cvtColor(whiteImg, whiteImg, COLOR_BGR2GRAY);
    threshold(yellowImg, thresh_yellow, 50, 255, THRESH_BINARY);
    threshold(whiteImg, thresh_white, 50, 255, THRESH_BINARY);

    vector<Point> left_line;
    vector<Point> right_line;

    // left road line
    for (int i = thresh_yellow.rows / 2; i < thresh_yellow.rows; i++)
    {
        for (int j = 0; j < thresh_yellow.cols / 2; j++)
        {
            if (thresh_yellow.at<uchar>(i, j) == 255)
            {
                left_line.push_back(Point(j, i));
                break;
            }
        }
    }
    // right road line
    for (int i = thresh_white.rows / 2; i < thresh_white.rows; i++)
    {
        for (int j = thresh_white.cols - 1; j > thresh_white.cols / 2; j--)
        {
            if (thresh_white.at<uchar>(i, j) == 255)
            {
                right_line.push_back(Point(j, i));
                break;
            }
        }
    }

    // draw the road line on the photo
    if (left_line.size() > 0 && right_line.size() > 0)
    {
        Point T_L = (left_line[0]);
        Point B_L = (left_line[left_line.size() - 1]);
        Point T_R = (right_line[0]);
        Point B_R = (right_line[right_line.size() - 1]);
        Point center_begin_point = Point((B_L.x + B_R.x) / 2, B_L.y);
        Point center_end_point = Point((T_L.x + T_R.x) / 2, T_L.y);
        slop = newSlop;
        newSlop = atan((center_begin_point.y - center_end_point.y) / (center_end_point.x - center_begin_point.x));

        // printf("b_l:%.2f,%.2f t_l:%.2f,%.2f\n", B_L.x, B_L.y, T_L.x, T_L.y);
        // printf("b_c : %.2f, %.2f t_c : %.2f, %.2f\n", center_begin_point.x, center_begin_point.y, center_end_point.x, center_end_point.y);
        // printf("b_r : %.2f, %.2f t_r : %.2f, %.2f\n", B_R.x, B_R.y, T_R.x, T_R.y);

        if (center_begin_point.y - center_end_point.y > 0)
            CAMERA1::isDetected = true;
    }
    else if (left_line.size() > 0 && right_line.size() == 0)
    {
        Point T_L = (left_line[0]);
        Point B_L = (left_line[left_line.size() - 1]);
        Point center_begin_point = Point(B_L.x, B_L.y);
        Point center_end_point = Point(T_L.x, T_L.y);
        slop = newSlop;
        newSlop = atan((center_begin_point.y - center_end_point.y) / (center_end_point.x - center_begin_point.x));

        // printf("b_c : %.2f, %.2f t_c : %.2f, %.2f\n", center_begin_point.x, center_begin_point.y, center_end_point.x, center_end_point.y);
        // printf("b_l : %.2f, %.2f t_l : %.2f, %.2f\n", B_L.x, B_L.y, T_L.x, T_L.y);

        if (center_begin_point.y - center_end_point.y > 0)
            CAMERA1::isDetected = true;
    }
    else if (left_line.size() == 0 && right_line.size() > 0)
    {
        Point T_R = (right_line[0]);
        Point B_R = (right_line[right_line.size() - 1]);
        Point center_begin_point = Point(B_R.x, B_R.y);
        Point center_end_point = Point(T_R.x, T_R.y);
        slop = newSlop;
        newSlop = atan((center_begin_point.y - center_end_point.y) / (center_end_point.x - center_begin_point.x));

        // printf("b_c : %.2f, %.2f t_c : %.2f, %.2f\n", center_begin_point.x, center_begin_point.y, center_end_point.x, center_end_point.y);
        // printf("b_r : %.2f, %.2f t_r : %.2f, %.2f\n", B_R.x, B_R.y, T_R.x, T_R.y);

        if (center_begin_point.y - center_end_point.y > 0)
            CAMERA1::isDetected = true;
    }
    else
    {
        slop = 3.141592653589793238462643383279502884 / 2;
        CAMERA1::isDetected = false;
    }
}