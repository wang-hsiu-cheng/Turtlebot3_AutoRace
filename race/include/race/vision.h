#ifndef _VISION_H_
#define _VISION_H_

#include <opencv2/opencv.hpp>
#include "ros/ros.h"

using namespace cv;

namespace VISION{
    struct table{
        float x_pixel;
        float y_pixel;
        float x_scara;
        float y_scara;
    };

    Point2f detect[3];  // 裝辨識出來的點：detect[0]裝E、detect[1]裝T、detect[2]裝L
    bool E_isDetected = false;
    bool T_isDetected = false;
    bool L_isDetected = false;

    bool E_isCatched = false;
    bool T_isCatched = false;
    bool L_isCatched = false;

    void E_image(void);  // 辨識E
    void CTFL_image(void);  // 辨識T、L
    void tf(void);  // 座標轉換

    void taking_photo(void);  //自動拍攝
    void init(ros::NodeHandle nh);

    /* internal function*/
    Point2f nearest_scara_point(Point2f input);
    int cmp(const void *a, const void *b);
    Mat E_filter(Mat img);
    void E_contour(Mat original_image, Mat image, double epsilon, \
        int minContour, int maxContour, double lowerBondArea);

    Mat CTFL_filter(Mat img);
    void CTFL_contour(Mat original_image, Mat image, double epsilon, \
        int minContour, int maxContour, double lowerBondArea);

    /* launch param*/
    double x_tf_cali = 0.9;
    double y_tf_cali = 0.9;
    double x_tf_intercept = -5;
    double y_tf_intercept = -5;
    
    double pixel_Xmin=150;
    double pixel_Xmax=390;
    double pixel_Ymax=470;
    double pixel_Ymin=35;
}

VISION::table COR[1200];

#endif
