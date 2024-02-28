#include "race/vision.h"
#include "math.h"
#include "ros/ros.h"

void VISION::init(ros::NodeHandle nh)
{
    nh.getParam("x_tf_cali", VISION::x_tf_cali);
    nh.getParam("y_tf_cali", VISION::y_tf_cali);
    nh.getParam("x_tf_intercept", VISION::x_tf_intercept);
    nh.getParam("y_tf_intercept", VISION::y_tf_intercept);

    nh.getParam("pixel_Xmin", VISION::pixel_Xmin);
    nh.getParam("pixel_Xmax", VISION::pixel_Xmax);
    nh.getParam("pixel_Ymax", VISION::pixel_Ymax);
    nh.getParam("pixel_Ymin", VISION::pixel_Ymin);
}

void VISION::takingPhoto(int imageName)
{
    VideoCapture cap(0); // 鏡頭編號依序從 012...
    Mat img;
    bool isPrinted = false;

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

    Mat original_image = img.clone();
    switch (imageName)
    {
    case 0:
        VISION::clock++;
        if (clock == 100)
            VISION::isDetected = true;
        break;
    case 1:
        VISION::green_light_image();
        break;
    case 2:
        VISION::warning_sign_image();
        break;
    case 3:
        img = VISION::filtGraph(img, 't');
        VISION::turnSignImage(original_image, img);
        break;
    case 4:
        VISION::parking_sign_image();
        break;
    case 5:
        VISION::stop_sign_image();
        break;
    case 6:
        VISION::fance_image();
        break;
    case 7:
        VISION::tunnel_sign_image();
        break;

    default:
        break;
    }
    return;
}
void VISION::green_light_image()
{
    // continue detecting red, yellow and green colors.
    // if green exist, then stop detecting and return.
}
void VISION::warning_sign_image()
{
    // continue detecting if sign exist.
    // if exist but not large enough, continue moving and detecting.
    // if exist and large enough, then stop detecting and return.
}
void VISION::fance_image()
{
}
void VISION::stop_sign_image()
{
}
void VISION::parking_sign_image()
{
}
void VISION::tunnel_sign_image()
{
}
Mat VISION::filtGraph(Mat img, char colorCode)
{
    Mat img_hsv, mask, result;
    cvtColor(img, img_hsv, COLOR_BGR2HSV);

    switch (colorCode)
    {
    case 'g':
    { // green range
        VISION::hue_m = 62;
        VISION::hue_M = 88;
        VISION::sat_m = 60;
        VISION::sat_M = 255;
        VISION::val_m = 137;
        VISION::val_M = 255;
    }
    break;
    case 'w':
    { // white range
        VISION::hue_m = 0;
        VISION::hue_M = 255;
        VISION::sat_m = 0;
        VISION::sat_M = 102;
        VISION::val_m = 147;
        VISION::val_M = 255;
    }
    break;
    case 'b':
    { // blue range
        VISION::hue_m = 100;
        VISION::hue_M = 137;
        VISION::sat_m = 151;
        VISION::sat_M = 255;
        VISION::val_m = 147;
        VISION::val_M = 255;
    }
    break;
    case 't':
    { // blue turn sign(draw) range
        VISION::hue_m = 89;
        VISION::hue_M = 123;
        VISION::sat_m = 132;
        VISION::sat_M = 255;
        VISION::val_m = 10;
        VISION::val_M = 255;
    }
    break;
    case 'l':
    { // road line(picture) range
        VISION::hue_m = 0;
        VISION::hue_M = 173;
        VISION::sat_m = 0;
        VISION::sat_M = 190;
        VISION::val_m = 191;
        VISION::val_M = 226;
    }
    break;

    default:
        break;
    }

    Scalar lower(hue_m, sat_m, val_m);
    Scalar upper(hue_M, sat_M, val_M);
    inRange(img_hsv, lower, upper, mask);

    result = Mat::zeros(img.size(), CV_8UC3);
    bitwise_and(img, img, result, mask);
    // imshow("Letter Filted", result);
    return result;
}
void VISION::turnSignImage(Mat original_image, Mat image)
{
    double epsilon = 6.5;      // DP Algorithm 的參數
    int minContour = 6;        // 邊數小於 minContour 會被遮罩
    int maxContour = 30;       // 邊數大於 maxContour 會遮罩
    double lowerBondArea = 20; // 面積低於 lowerBondArea 的輪廓會被遮罩

    cvtColor(image, image, COLOR_BGR2GRAY);
    threshold(image, image, 40, 255, THRESH_BINARY);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    // 1) 找出邊緣
    findContours(image, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);
    // imshow("Contours Image (before DP)", image);

    vector<vector<Point>> polyContours(contours.size()); // polyContours 用來存放折線點的集合

    // 2) 簡化邊緣： DP Algorithm
    for (size_t i = 0; i < contours.size(); i++)
    {
        approxPolyDP(Mat(contours[i]), polyContours[i], epsilon, true);
    }

    Mat dp_image = Mat::zeros(image.size(), CV_8UC3); // 初始化 Mat 後才能使用 drawContours
    drawContours(dp_image, polyContours, -1, Scalar(255, 0, 255), 1, 0);
    // imshow("Contours Image (1):", dp_image);

    // 3) 過濾不好的邊緣，用 badContour_mask 遮罩壞輪廓
    Mat badContour_mask = Mat::zeros(image.size(), CV_8UC3);
    for (size_t a = 0; a < polyContours.size(); a++)
    {
        // if 裡面如果是 true 代表該輪廓是不好的，會先被畫在 badContour)mask 上面
        if (polyContours[a].size() < minContour || polyContours[a].size() > maxContour ||
            contourArea(polyContours[a]) < lowerBondArea)
        {
            for (size_t b = 0; b < polyContours[a].size() - 1; b++)
            {
                line(badContour_mask, polyContours[a][b], polyContours[a][b + 1], Scalar(0, 255, 0), 3);
            }
            line(badContour_mask, polyContours[a][0], polyContours[a][polyContours[a].size() - 1], Scalar(0, 255, 0), 1, LINE_AA);
        }
    }

    // 進行壞輪廓的遮罩
    Mat dp_optim_v1_image = Mat::zeros(image.size(), CV_8UC3);

    cvtColor(badContour_mask, badContour_mask, COLOR_BGR2GRAY);
    threshold(badContour_mask, badContour_mask, 0, 255, THRESH_BINARY_INV);
    bitwise_and(dp_image, dp_image, dp_optim_v1_image, badContour_mask);
    // imshow("DP image (Optim v1): ", dp_optim_v1_image);

    // 4) 再從好的邊緣圖中找出邊緣
    cvtColor(dp_optim_v1_image, dp_optim_v1_image, COLOR_BGR2GRAY);
    threshold(dp_optim_v1_image, dp_optim_v1_image, 0, 255, THRESH_BINARY);
    vector<vector<Point>> contours2;
    vector<Vec4i> hierarchy2;

    findContours(dp_optim_v1_image, contours2, hierarchy2, RETR_LIST, CHAIN_APPROX_SIMPLE);

    // 5) 簡化好輪廓 DP演算法
    vector<vector<Point>> polyContours2(contours2.size()); // 存放折線點的集合
    Mat dp_image_2 = Mat::zeros(dp_optim_v1_image.size(), CV_8UC3);

    for (size_t i = 0; i < contours2.size(); i++)
    {
        approxPolyDP(Mat(contours2[i]), polyContours2[i], epsilon, true);
    }

    drawContours(dp_image_2, polyContours2, 0, Scalar(255, 0, 255), 2, 0);
    Mat dp_image_text = dp_image_2.clone();
    // imshow("Contours Image (2):", dp_image_text);
    // drawContours(dp_image_text, polyContours2, 0, Scalar(255, 0, 255), 1, 0);

    // 7) 擬和旋轉矩形 + 邊長數量判斷字型 + 標示方塊中心點
    RotatedRect box;     // 旋轉矩形 class
    Point2f vertices[4]; // 旋轉矩形四頂點
    vector<Point> pt;    // 存一個contour中的點集合
    int left_pt_count = 0;
    int right_pt_count = 0;
    int contourNumbers = polyContours2.size();
    contourNumbers = (contourNumbers > 0) ? 1 : 0;

    for (int a = 0; a < contourNumbers; a++)
    {
        // A) 旋轉矩形
        pt.clear();
        for (int b = 0; b < polyContours2[a].size(); b++)
        {
            pt.push_back(polyContours2[a][b]);
        }
        box = minAreaRect(pt); // 找到最小矩形，存到 box 中
        box.points(vertices);  // 把矩形的四個頂點資訊丟給 vertices，points()是 RotatedRect 的函式

        for (int i = 0; i < 4; i++)
        {
            line(dp_image_2, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 2); // 描出旋轉矩形
        }
        // 標示
        circle(dp_image_2, (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, 0, Scalar(0, 255, 255), 8);     // 繪製中心點
        circle(original_image, (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, 0, Scalar(0, 255, 255), 8); // 與原圖比較

        for (int b = 0; b < polyContours2[a].size(); b++)
        {
            circle(dp_image_2, polyContours2[a][b], 0, Scalar(0, 255, 255), 4);
            if (polyContours2[a][b].x < ((vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4).x)
            {
                left_pt_count++;
            }
            else if (polyContours2[a][b].x > ((vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4).x)
            {
                right_pt_count++;
            }
        }
        string direction = (left_pt_count < right_pt_count) ? "turn left" : "turn right";
        putText(original_image, direction, Point(10, 25), 0, 0.8, Scalar(0, 255, 0), 1, 1, false);
    }
    if (left_pt_count < 5)
        VISION::isDetected = false;
    else
        VISION::isDetected = true;
    // imshow("D", dp_image_2);
    // imshow("camera", original_image);
}
void VISION::road_line_image(Mat src, Mat &ROI, bool isPrinted)
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
        VISION::isDetected = true;
    }
}

// void Callback(const std_msgs::Int64::ConstPtr &msg)
// {
//     std::cout << "msg.data = " << msg->data << "\n";
//     VISION::lastImageName = imageName;
//     VISION::imageName = msg->data;
//     if ((lastImageName != imageName && imageName != 100) && !VISION::isDetected)
//     {
//         VISION::takingPhoto(imageName);
//         pub.publish(false);
//     }
//     else if ((lastImageName == imageName || imageName == 100) && !VISION::isDetected)
//     {
//         pub.publish(false);
//     }
//     else
//     {
//         pub.publish(true);
//     }
// }
// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "vision");
//     ros::NodeHandle nh;

//     VISION::init(ros::NodeHandle nh);

//     ros::Subscriber sub = nh.subscribe("camera", 10, Callback);
//     ros::Publisher pub = nh.advertise<bool>("run_fromCamera", 10);
//     ros::Rate loop_rate(1);

//     while (ros::ok())
//     {
//         ros::spinOnce();
//         loop_rate.sleep();
//     }
//     return 0;
// }
void VISION::E_image(void)
{
    //     const double epsilon = 5;       // DP Algorithm 的參數
    //     const int minContour = 3;       // 邊數小於 minContour 會被遮罩
    //     const int maxContour = 6;       // 邊數大於 maxContour
    //     const double lowerBondArea = 5; // 面積低於 lowerBondArea 的輪廓會被遮罩

    //     std::string path = PATH;
    //     Mat src = imread(PATH);

    //     // resize(src, src, Size(src.cols/2, src.rows/2));
    //     Mat original_image = src.clone();

    //     src = VISION::E_filter(src);
    //     VISION::E_contour(original_image, src, epsilon, minContour, maxContour, lowerBondArea);
}
void VISION::CTFL_image(void)
{
    //     /// 需要調整的變數
    //     const double epsilon = 3;        // DP Algorithm 的參數
    //     const int minContour = 4;        // 邊數小於 minContour 會被遮罩
    //     const int maxContour = 9;        // 邊數大於 maxContour 會遮罩
    //     const double lowerBondArea = 20; // 面積低於 lowerBondArea 的輪廓會被遮罩
    //                                      ///

    //     std::string path = PATH;
    //     Mat src = imread(PATH);

    //     // resize(src, src, Size(src.cols, src.rows));
    //     Mat original_image = src.clone();
    //     // imshow("original",original_image);

    //     src = VISION::CTFL_filter(src);
    //     VISION::CTFL_contour(original_image, src, epsilon, minContour, maxContour, lowerBondArea);
}
Mat VISION::E_filter(Mat img)
{
    //     Mat img_hsv, mask, result;
    //     cvtColor(img, img_hsv, COLOR_BGR2HSV);

    //     // green range
    //     const int hue_m = 62;
    //     const int hue_M = 88;
    //     const int sat_m = 60;
    //     const int sat_M = 255;
    //     const int val_m = 137;
    //     const int val_M = 255;

    //     Scalar lower(hue_m, sat_m, val_m);
    //     Scalar upper(hue_M, sat_M, val_M);
    //     inRange(img_hsv, lower, upper, mask);

    //     result = Mat::zeros(img.size(), CV_8UC3);
    //     bitwise_and(img, img, result, mask);
    //     // imshow("Letter Filted", result);
    //     // waitKey(0);
    //      return result;
    return img;
}
void VISION::E_contour(Mat original_image, Mat image, double epsilon,
                       int minContour, int maxContour, double lowerBondArea)
{
    //     cvtColor(image, image, COLOR_BGR2GRAY);
    //     threshold(image, image, 40, 255, THRESH_BINARY);

    //     std::vector<std::vector<Point>> contours;
    //     std::vector<Vec4i> hierarchy;

    //     // 1) 找出邊緣
    //     findContours(image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    //     // imshow("A", image);

    //     std::vector<std::vector<Point>> polyContours(contours.size()); // polyContours 用來存放折線點的集合

    //     // 2) 簡化邊緣： DP Algorithm
    //     for (size_t i = 0; i < contours.size(); i++)
    //     {
    //         approxPolyDP(Mat(contours[i]), polyContours[i], epsilon, true);
    //     }

    //     Mat dp_image = Mat::zeros(image.size(), CV_8UC3); // 初始化 Mat 後才能使用 drawContours
    //     drawContours(dp_image, polyContours, -2, Scalar(255, 0, 255), 1, 0);
    //     // imshow("B", dp_image);

    //     // 3) 過濾不好的邊緣，用 badContour_mask 遮罩壞輪廓
    //     Mat badContour_mask = Mat::zeros(image.size(), CV_8UC3);
    //     for (size_t a = 0; a < polyContours.size(); a++)
    //     {
    //         // if 裡面如果是 true 代表該輪廓是不好的，會先被畫在 badContour_mask 上面
    //         if (polyContours[a].size() < minContour || polyContours[a].size() > maxContour ||
    //             contourArea(polyContours[a]) < lowerBondArea)
    //         {
    //             for (size_t b = 0; b < polyContours[a].size() - 1; b++)
    //             {
    //                 line(badContour_mask, polyContours[a][b], polyContours[a][b + 1], Scalar(0, 255, 0), 3);
    //             }
    //             line(badContour_mask, polyContours[a][0], polyContours[a][polyContours[a].size() - 1], Scalar(0, 255, 0), 1, LINE_AA);
    //         }
    //     }

    //     // 4) 進行壞輪廓的遮罩
    //     Mat dp_optim_v1_image = Mat::zeros(image.size(), CV_8UC3);

    //     cvtColor(badContour_mask, badContour_mask, COLOR_BGR2GRAY);
    //     threshold(badContour_mask, badContour_mask, 0, 255, THRESH_BINARY_INV);
    //     bitwise_and(dp_image, dp_image, dp_optim_v1_image, badContour_mask);
    //     // imshow("C", dp_optim_v1_image); waitKey(0);

    //     // 5) 再從好的邊緣圖中找出邊緣
    //     cvtColor(dp_optim_v1_image, dp_optim_v1_image, COLOR_BGR2GRAY);
    //     threshold(dp_optim_v1_image, dp_optim_v1_image, 0, 255, THRESH_BINARY);
    //     std::vector<std::vector<Point>> contours2;
    //     std::vector<Vec4i> hierarchy2;

    //     findContours(dp_optim_v1_image, contours2, hierarchy2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    //     // 6) 簡化好輪廓 DP演算法
    //     std::vector<std::vector<Point>> polyContours2(contours2.size()); // 存放折線點的集合
    //     Mat dp_image_2 = Mat::zeros(dp_optim_v1_image.size(), CV_8UC3);
    //     for (size_t i = 0; i < contours2.size(); i++)
    //     {
    //         approxPolyDP(Mat(contours2[i]), polyContours2[i], epsilon, true);
    //     }
    //     drawContours(dp_image_2, polyContours2, -2, Scalar(255, 0, 255), 1, 0);

    //     // 7) 擬和旋轉矩形 + 標示方塊中心點
    //     RotatedRect box;       // 旋轉矩形 class
    //     Point2f vertices[4];   // 旋轉矩形四頂點
    //     std::vector<Point> pt; // 存一個contour中的點集合

    //     for (int a = 0; a < polyContours2.size(); a++)
    //     {
    //         pt.clear();
    //         for (int b = 0; b < polyContours2[a].size(); b++)
    //         {
    //             pt.push_back(polyContours2[a][b]);
    //         }

    //         box = minAreaRect(pt); // 找到最小矩形，存到 box 中
    //         box.points(vertices);  // 把矩形的四個頂點資訊丟給 vertices，points()是 RotatedRect 的函式

    //         double E_x, E_y;
    //         E_x = (vertices[0].x + vertices[1].x + vertices[2].x + vertices[3].x) / 4;
    //         E_y = (vertices[0].y + vertices[1].y + vertices[2].y + vertices[3].y) / 4;

    //         if (E_x >= pixel_Xmin && E_x <= pixel_Xmax && E_y >= pixel_Ymin && E_y <= pixel_Ymax)
    //         {
    //             for (int i = 0; i < 4; i++)
    //             {
    //                 line(dp_image_2, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 2); // 描出旋轉矩形
    //             }

    //             circle(dp_image_2, (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, 0, Scalar(0, 255, 255), 8);     // 繪製中心點
    //             circle(original_image, (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, 0, Scalar(0, 255, 255), 8); // 放回原圖比較

    //             putText(dp_image_2, "E", (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, 1, 3, Scalar(0, 0, 255), 2);
    //             putText(original_image, "E", (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, 1, 3, Scalar(0, 0, 255), 2);

    //             if (!E_isDetected)
    //             {
    //                 E_isDetected = true;
    //                 VISION::detect[1] = (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4;

    //                 std::cout << "\n-- Point E (No TF)--\nX: " << (vertices[0].x + vertices[1].x + vertices[2].x + vertices[3].x) / 4
    //                           << "\nY: " << (vertices[0].y + vertices[1].y + vertices[2].y + vertices[3].y) / 4 << std::endl;
    //             }
    //         }
    //     }
    //     // imshow("D", dp_image_2);
    //     // imshow("E", original_image);
    //     // waitKey(0);
    //     imwrite("/home/ditrobotics/TEL/src/race/src/E.png", original_image);
}
Mat VISION::CTFL_filter(Mat img)
{
    //     Mat img_hsv, mask, result;
    //     cvtColor(img, img_hsv, COLOR_BGR2HSV);

    //     // range 2 (light)
    //     int hue_m = 82;
    //     int hue_M = 110;
    //     int sat_m = 73;
    //     int sat_M = 255;
    //     int val_m = 255;
    //     int val_M = 255;

    //     Scalar lower(hue_m, sat_m, val_m);
    //     Scalar upper(hue_M, sat_M, val_M);
    //     inRange(img_hsv, lower, upper, mask);

    //     result = Mat::zeros(img.size(), CV_8UC3);
    //     bitwise_and(img, img, result, mask);
    //     imshow("Letter Filted", result);
    //     return result;
    return img;
}
void VISION::CTFL_contour(Mat original_image, Mat image, double epsilon,
                          int minContour, int maxContour, double lowerBondArea)
{
    //     cvtColor(image, image, COLOR_BGR2GRAY);
    //     threshold(image, image, 40, 255, THRESH_BINARY);

    //     std::vector<std::vector<Point>> contours;
    //     std::vector<Vec4i> hierarchy;

    //     // 1) 找出邊緣
    //     findContours(image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    //     // imshow("Contours Image (before DP)", image);

    //     std::vector<std::vector<Point>> polyContours(contours.size()); // polyContours 用來存放折線點的集合

    //     // 2) 簡化邊緣： DP Algorithm
    //     for (size_t i = 0; i < contours.size(); i++)
    //     {
    //         approxPolyDP(Mat(contours[i]), polyContours[i], epsilon, true);
    //     }

    //     Mat dp_image = Mat::zeros(image.size(), CV_8UC3); // 初始化 Mat 後才能使用 drawContours
    //     drawContours(dp_image, polyContours, -2, Scalar(255, 0, 255), 1, 0);
    //     // imshow("Contours Image (After DP):", dp_image);

    //     // 3) 過濾不好的邊緣，用 badContour_mask 遮罩壞輪廓
    //     Mat badContour_mask = Mat::zeros(image.size(), CV_8UC3);
    //     for (size_t a = 0; a < polyContours.size(); a++)
    //     {
    //         // if 裡面如果是 true 代表該輪廓是不好的，會先被畫在 badContour)mask 上面
    //         if (polyContours[a].size() < minContour || polyContours[a].size() > maxContour ||
    //             contourArea(polyContours[a]) < lowerBondArea)
    //         {
    //             for (size_t b = 0; b < polyContours[a].size() - 1; b++)
    //             {
    //                 line(badContour_mask, polyContours[a][b], polyContours[a][b + 1], Scalar(0, 255, 0), 3);
    //             }
    //             line(badContour_mask, polyContours[a][0], polyContours[a][polyContours[a].size() - 1], Scalar(0, 255, 0), 1, LINE_AA);
    //         }
    //     }

    //     // 進行壞輪廓的遮罩
    //     Mat dp_optim_v1_image = Mat::zeros(image.size(), CV_8UC3);

    //     cvtColor(badContour_mask, badContour_mask, COLOR_BGR2GRAY);
    //     threshold(badContour_mask, badContour_mask, 0, 255, THRESH_BINARY_INV);
    //     bitwise_and(dp_image, dp_image, dp_optim_v1_image, badContour_mask);
    //     // imshow("DP image (Optim v1): ", dp_optim_v1_image);
    //     // waitKey(0);

    //     // 4) 再從好的邊緣圖中找出邊緣
    //     cvtColor(dp_optim_v1_image, dp_optim_v1_image, COLOR_BGR2GRAY);
    //     threshold(dp_optim_v1_image, dp_optim_v1_image, 0, 255, THRESH_BINARY);
    //     std::vector<std::vector<Point>> contours2;
    //     std::vector<Vec4i> hierarchy2;

    //     findContours(dp_optim_v1_image, contours2, hierarchy2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    //     // 5) 簡化好輪廓 DP演算法
    //     std::vector<std::vector<Point>> polyContours2(contours2.size()); // 存放折線點的集合
    //     Mat dp_image_2 = Mat::zeros(dp_optim_v1_image.size(), CV_8UC3);
    //     for (size_t i = 0; i < contours2.size(); i++)
    //     {
    //         approxPolyDP(Mat(contours2[i]), polyContours2[i], epsilon, true);
    //     }
    //     drawContours(dp_image_2, polyContours2, -2, Scalar(255, 0, 255), 1, 0);

    //     Mat dp_image_text = dp_image_2.clone();
    //     // imshow("Contours Image (After DP):", dp_image_text);

    //     // 7) 擬和旋轉矩形 + 邊長數量判斷字型 + 標示方塊中心點
    //     RotatedRect box;       // 旋轉矩形 class
    //     Point2f vertices[4];   // 旋轉矩形四頂點
    //     std::vector<Point> pt; // 存一個contour中的點集合

    //     for (int a = 0; a < polyContours2.size(); a++)
    //     {
    //         // A) 旋轉矩形
    //         pt.clear();
    //         for (int b = 0; b < polyContours2[a].size(); b++)
    //         {
    //             pt.push_back(polyContours2[a][b]);
    //         }
    //         box = minAreaRect(pt); // 找到最小矩形，存到 box 中
    //         box.points(vertices);  // 把矩形的四個頂點資訊丟給 verti    ces，points()是 RotatedRect 的函式

    //         double CTFL_x, CTFL_y;
    //         CTFL_x = (vertices[0].x + vertices[1].x + vertices[2].x + vertices[3].x) / 4;
    //         CTFL_y = (vertices[0].y + vertices[1].y + vertices[2].y + vertices[3].y) / 4;

    //         if (CTFL_x >= pixel_Xmin && CTFL_x <= pixel_Xmax && CTFL_y >= pixel_Ymin && CTFL_y <= pixel_Ymax)
    //         {
    //             for (int i = 0; i < 4; i++)
    //             {
    //                 line(dp_image_2, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 2); // 描出旋轉矩形
    //             }

    //             // 標示
    //             circle(dp_image_2, (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, 0, Scalar(0, 255, 255), 8);     // 繪製中心點
    //             circle(original_image, (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, 0, Scalar(0, 255, 255), 8); // 與原圖比較

    //             // B) 判斷字母(用邊長個數篩選)
    //             if (polyContours2[a].size() == 6 || polyContours2[a].size() == 7)
    //             { // L
    //                 // 標示
    //                 putText(dp_image_2, "L", (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, 1, 3, Scalar(0, 255, 255), 3);
    //                 putText(original_image, "L", (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, 1, 3, Scalar(0, 0, 255), 2);

    //                 if (L_isDetected == false)
    //                 {
    //                     L_isDetected = true;
    //                     VISION::detect[2] = (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4;

    //                     std::cout << "\n-- Point L --\nX: " << (vertices[0].x + vertices[1].x + vertices[2].x + vertices[3].x) / 4
    //                               << "\nY: " << (vertices[0].y + vertices[1].y + vertices[2].y + vertices[3].y) / 4 << std::endl;
    //                 }
    //             }

    //             if (polyContours2[a].size() == 8 || polyContours2[a].size() == 9)
    //             { // T、E (此時場上不會有 E)
    //                 // 標示
    //                 putText(dp_image_2, "T", (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, 1, 3, Scalar(0, 255, 255), 3);
    //                 putText(original_image, "T", (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, 1, 3, Scalar(0, 0, 255), 2);

    //                 if (T_isDetected == false)
    //                 {
    //                     T_isDetected = true;
    //                     VISION::detect[0] = (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4;

    //                     std::cout << "\n-- Point T --\nX: " << (vertices[0].x + vertices[1].x + vertices[2].x + vertices[3].x) / 4
    //                               << "\nY: " << (vertices[0].y + vertices[1].y + vertices[2].y + vertices[3].y) / 4 << std::endl;
    //                 }
    //             }
    //         }
    //     }

    //     // imshow("Contours Filted", dp_image_2);
    //     // imshow("Original Image (highlight)", original_image);
    //     // waitKey(0);

    //     imwrite("/home/ditrobotics/TEL/src/race/src/CTFL.png", original_image);
}
Point2f VISION::nearest_scara_point(Point2f input)
{
    //     Point2f temp(0, 0);

    //     float distance[1121] = {0};

    //     for (int i = 0; i < 1121; i++)
    //         distance[i] = sqrt((input.x - COR[i].x_pixel) * (input.x - COR[i].x_pixel) + (input.y - COR[i].y_pixel) * (input.y - COR[i].y_pixel));

    //     int min = 0;
    //     for (int i = 0; i < 1121; i++)
    //         if (distance[i] < distance[min])
    //             min = i;

    //     printf("\n=== Before Cali ===\nX: %lf\nY: %lf\n", COR[min].x_scara, COR[min].y_scara);
    //     temp.x = (COR[min].x_scara * VISION::x_tf_cali) + x_tf_intercept;
    //     temp.y = (COR[min].y_scara * VISION::y_tf_cali) + y_tf_intercept;
    //     return temp;
    return input;
}
void VISION::tf(void)
{
    // FILE *file;
    // file = fopen(csv_file, "r");

    // int records = 0;
    // int row = 0;
    // do
    // {
    //     records = fscanf(file, "%f%f%f%f\n", &COR[row].x_pixel, &COR[row].y_pixel, &COR[row].x_scara, &COR[row].y_scara);
    //     if (records == 4)
    //         row++;
    //     else
    //         return;
    // } while (!feof(file) && ros::ok());
    // fclose(file);
    // file = NULL;

    // if (T_isDetected && !T_isCatched)
    //     VISION::detect[0] = nearest_scara_point(VISION::detect[0]);
    // if (E_isDetected && !E_isCatched)
    //     VISION::detect[1] = nearest_scara_point(VISION::detect[1]);
    // if (L_isDetected && !L_isCatched)
    //     VISION::detect[2] = nearest_scara_point(VISION::detect[2]);

    // for (int i = 0; i < 3; i++)
    //     std::cout << "\n(tf) detect[" << i << "]: " << VISION::detect[i] << std::endl;
}