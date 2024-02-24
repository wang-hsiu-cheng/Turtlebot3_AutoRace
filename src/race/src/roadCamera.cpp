#include "race/roadCamera.h"
#include "math.h"
#include "ros/ros.h"

void CAMERA1::detectRoad()
{
    VideoCapture cap(0); // 鏡頭編號依序從 012...
    Mat img;
    bool isPrinted = false;

    if (!cap.isOpened())
    { // 確認有連線到該編號的鏡頭
        cout << "Cannot open capture\n";
    }
    bool ret = cap.read(img);
    if (!ret)
    {
        cout << "Cant receive frame\n";
        break;
    }

    Mat originalImage = img.clone();
    img = filtGraph(img);
    roadLineImage(originalImage, img, isPrinted);

    if (waitKey(1) == 'q')
        break;
    return;
}

Mat CAMERA1::filtGraph(Mat img)
{
    Mat img_hsv, mask, result;
    cvtColor(img, img_hsv, COLOR_BGR2HSV);

    CAMERA1::hue_m = 0;
    CAMERA1::hue_M = 173;
    CAMERA1::sat_m = 0;
    CAMERA1::sat_M = 190;
    CAMERA1::val_m = 191;
    CAMERA1::val_M = 226;

    Scalar lower(hue_m, sat_m, val_m);
    Scalar upper(hue_M, sat_M, val_M);
    inRange(img_hsv, lower, upper, mask);

    result = Mat::zeros(img.size(), CV_8UC3);
    bitwise_and(img, img, result, mask);

    return result;
}

void CAMERA1::roadLineImage(Mat src, Mat &ROI, bool isPrinted)
{
    Mat gray;
    cvtColor(ROI, ROI, COLOR_BGR2GRAY);

    Mat thresh;
    threshold(ROI, thresh, 50, 255, THRESH_BINARY);
    // imshow("gray", thresh);

    vector<Point> left_line;
    vector<Point> right_line;

    // left road line
    for (int i = thresh.rows / 2; i < thresh.rows; i++)
    {
        for (int j = 0; j < thresh.cols / 2; j++)
        {
            if (thresh.at<uchar>(i, j) == 255)
            {
                left_line.push_back(Point(j, i));
                break;
            }
        }
    }
    // right road line
    for (int i = thresh.rows / 2; i < thresh.rows; i++)
    {
        for (int j = thresh.cols - 1; j > thresh.cols / 2; j--)
        {
            if (thresh.at<uchar>(i, j) == 255)
            {
                right_line.push_back(Point(j, i));
                break;
            }
        }
    }

    // draw the road line on the photo
    if (left_line.size() > 0 && right_line.size() > 0)
    {
        double times = pow(10, 2);
        Point T_L = (left_line[0]);
        Point B_L = (left_line[left_line.size() - 1]);
        Point T_R = (right_line[0]);
        Point B_R = (right_line[right_line.size() - 1]);
        Point center_begin_point = Point((B_L.x + B_R.x) / 2, B_L.y);
        Point center_end_point = Point((T_L.x + T_R.x) / 2, T_L.y);
        double slop = (center_begin_point.y - center_end_point.y) / (center_end_point.x - center_begin_point.x);
        // if (!isPrinted)
        // {
        //     isPrinted = true;
        //     printf("b_l:%.2f,%.2f t_l:%.2f,%.2f\n", B_L.x * times, B_L.y * times, T_L.x * times, T_L.y * times);
        //     printf("b_c : %.2f, %.2f t_c : %.2f, %.2f\n", center_begin_point.x * times, center_begin_point.y * times, center_end_point.x * times, center_end_point.y * times);
        //     printf("b_r : %.2f, %.2f t_r : %.2f, %.2f\n", B_R.x * times, B_R.y * times, T_R.x * times, T_R.y * times);
        // }

        // circle(src, B_L, 10, Scalar(0, 0, 255), -1);
        // circle(src, T_L, 10, Scalar(0, 255, 0), -1);
        // circle(src, T_R, 10, Scalar(255, 0, 0), -1);
        // circle(src, B_R, 10, Scalar(0, 255, 255), -1);
        // circle(src, center_begin_point, 10, Scalar(255, 0, 0), -1);
        // circle(src, center_end_point, 10, Scalar(0, 255, 255), -1);

        // vector<Point> pts;
        // pts = {B_L, T_L, T_R, B_R};
        // vector<vector<Point>> ppts;
        // ppts.push_back(pts);
        // fillPoly(src, ppts, Scalar(100, 201, 201));

        // line(src, Point(B_L), Point(T_L), Scalar(0, 255, 0), 10);
        // line(src, Point(B_R), Point(T_R), Scalar(0, 255, 0), 10);
        // line(src, Point(center_begin_point), Point(center_end_point), Scalar(0, 255, 255), 10);
        if (center_begin_point.y - center_end_point.y > 0)
            CAMERA1::isDetected = true;
    }
}