#ifndef _VISION_H_
#define _VISION_H_

#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

namespace VISION
{

    int lastImageName = 100;
    int imageName = 100;
    int clock = 0;
    bool is_sign_exist = false;
    bool isDetected = false;

    void road_line_image(Mat src, Mat &ROI, bool isPrinted);
    void green_light_image();
    void warning_sign_image();
    void turnSignImage(Mat original_image, Mat image);
    void stop_sign_image();
    void parking_sign_image();
    void tunnel_sign_image();
    void fance_image();
    void init(ros::NodeHandle nh);
    void takingPhoto(int imageName); // 自動拍攝
    Mat filtGraph(Mat img, char color_code);
    /* launch param*/
    int hue_m = 0;
    int hue_M = 255;
    int sat_m = 0;
    int sat_M = 255;
    int val_m = 0;
    int val_M = 255;

    struct table
    {
        float x_pixel;
        float y_pixel;
        float x_scara;
        float y_scara;
    };
    void tf(void); // 座標轉換
    Point2f nearest_scara_point(Point2f input);
    int cmp(const void *a, const void *b);
    void E_image(void);    // 辨識E
    void CTFL_image(void); // 辨識T、L
    Mat E_filter(Mat img);
    void E_contour(Mat original_image, Mat image, double epsilon,
                   int minContour, int maxContour, double lowerBondArea);
    Mat CTFL_filter(Mat img);
    void CTFL_contour(Mat original_image, Mat image, double epsilon,
                      int minContour, int maxContour, double lowerBondArea);
    double x_tf_cali = 0.9;
    double y_tf_cali = 0.9;
    double x_tf_intercept = -5;
    double y_tf_intercept = -5;
    double pixel_Xmin = 150;
    double pixel_Xmax = 390;
    double pixel_Ymax = 470;
    double pixel_Ymin = 35;
    double slop = 0;
    vector<double> point_left(2);
    vector<double> vector_left(2);
    vector<double> point_right(2);
    vector<double> vector_right(2);
    Point2f detect[3]; // 裝辨識出來的點：detect[0]裝E、detect[1]裝T、detect[2]裝L
    bool E_isDetected = false;
    bool T_isDetected = false;
    bool L_isDetected = false;

    bool E_isCatched = false;
    bool T_isCatched = false;
    bool L_isCatched = false;
}

VISION::table COR[1200];

#endif
