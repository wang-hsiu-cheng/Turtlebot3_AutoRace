#include "race/vision.h"
#include "math.h"
#include <time.h>
#include "ros/ros.h"

void VISION::init(ros::NodeHandle nh)
{
}

void VISION::takingPhoto(int imageName)
{
    VideoCapture cap(1); // 鏡頭編號依序從 012...
    Mat img;
    VISION::isDetected = false;

    printf("[INFO] start takingPhoto\n");

    if (!cap.isOpened())
    { // 確認有連線到該編號的鏡頭
        cout << "Cannot open capture\n";
        return;
    }
    for (int i = 0; i < detectingLoop; i++)
    {
        bool ret = cap.read(img);
        while (!ret)
        {
            cout << "Cant receive frame\n";
            ret = cap.read(img);
        }
    }
    Mat original_image = img.clone();
    switch (imageName)
    {
    case 0:
        counter++;
        if (counter == 100)
            VISION::isDetected = true;
        break;
    case 1:
        // img = VISION::filtGraph(img, GREEN);
        // VISION::greenLightImage(original_image, img);
        break;
    case 2:
        img = VISION::filtGraph(img, RED1);
        VISION::warnSignImage(original_image, img);
        break;
    case 3:
        img = VISION::filtGraph(img, BLUE);
        VISION::turnSignImage(original_image, img);
        break;
    case 4:
        img = VISION::filtGraph(img, RED2);
        VISION::fanceImage(original_image, img);
        break;
    case 5:
        VISION::stop_sign_image();
        break;
    case 6:
        VISION::parking_sign_image();
        break;
    default:
        break;
    }
    // if (detectedCounter >= detectingLoop / 2)
    // {
    //     VISION::isDetected = true;
    //     cout << "detected";
    // }
    if (detectedCounter > 0)
    {
        VISION::isDetected = true;
        cout << "detected";
    }
    detectedCounter = 0;
    return;
}
void VISION::DontDetectAnything()
{
    // double start, end = 10 * CLOCKS_PER_SEC;
    // // 紀錄開始計時的時間
    // start = clock();
    // while (clock() - start >= end)
    // {
    //     VISION::isDetected = true;
    // }
    // return;
}
void VISION::greenLightImage(Mat original_image, Mat image)
{
    // continue detecting red, yellow and green colors.
    // if green exist, then stop detecting and return.
}
void VISION::warnSignImage(Mat original_image, Mat image)
{
    double epsilon = 12;
    int minContour = 3;
    int maxContour = 5;
    double lowerBondArea = 1000;
    int triangleCount = 0;
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
    drawContours(dp_image, polyContours, -1, Scalar(255, 0, 255), 0, 0);
    // imshow("Contours Image (1):", dp_image);

    // 3) 過濾不好的邊緣，用 badContour_mask 遮罩壞輪廓
    Mat badContour_mask = Mat::zeros(image.size(), CV_8UC3);

    double largestArea = contourArea(polyContours[0]);
    for (size_t a = 0; a < polyContours.size(); a++)
    {
        if (largestArea < contourArea(polyContours[a]))
            largestArea = contourArea(polyContours[a]);
    }
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

    findContours(dp_optim_v1_image, contours2, hierarchy2, RETR_LIST, CHAIN_APPROX_NONE);

    // 5) 簡化好輪廓 DP演算法
    vector<vector<Point>> polyContours2(contours2.size()); // 存放折線點的集合
    Mat dp_image_2 = Mat::zeros(dp_optim_v1_image.size(), CV_8UC3);

    for (size_t i = 0; i < contours2.size(); i++)
    {
        approxPolyDP(Mat(contours2[i]), polyContours2[i], 1, true);
    }
    // cout << polyContours2.size();
    drawContours(dp_image_2, polyContours2, -1, Scalar(255, 0, 255), 2, 0);
    Mat dp_image_text = dp_image_2.clone();
    // imshow("Contours Image (2):", dp_image_text);
    // drawContours(dp_image_text, polyContours2, 0, Scalar(255, 0, 255), 1, 0);

    // 7) 擬和旋轉矩形 + 邊長數量判斷字型 + 標示方塊中心點
    RotatedRect box;     // 旋轉矩形 class
    Point2f vertices[4]; // 旋轉矩形四頂點
    vector<Point> pt;    // 存一個contour中的點集合
    int leftPoints = 0;
    int rightPoints = 0;
    int contourNumbers = polyContours2.size();
    // contourNumbers = (contourNumbers > 0) ? 1 : 0;

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
        // cout << polyContours2[a].size() << endl;
        if (polyContours2[a].size() == 3)
        {
            triangleCount++;
        }
        // for (int i = 0; i < 4; i++)
        // {
        //     line(dp_image_2, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 2); // 描出旋轉矩形
        // }
        // 標示
        circle(dp_image_2, (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, 0, Scalar(0, 255, 255), 8);     // 繪製中心點
        circle(original_image, (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, 0, Scalar(0, 255, 255), 8); // 與原圖比較
    }
    // imshow("contour info", contours_info(dp_image_text, polyContours2));

    // imshow("D", dp_image_2);
    // imshow("camera", original_image);
    if (triangleCount < 3 && triangleCount > 0)
    {
        // cout << "Triangle Count: " << rectangleCount;
        detectedCounter++;
        // cout << "detect count: " << detect << endl;
    }
    // continue detecting if sign exist.
    // if exist but not large enough, continue moving and detecting.
    // if exist and large enough, then stop detecting and return.
    return;
}
void VISION::fanceImage(Mat original_image, Mat image)
{
    double epsilon = 20;
    int minContour = 3;
    int maxContour = 5;
    double lowerBondArea = 200;
    double angle = 400;
    // int rectangleCount = 0;
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
    drawContours(dp_image, polyContours, -1, Scalar(255, 0, 255), 0, 0);
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

    findContours(dp_optim_v1_image, contours2, hierarchy2, RETR_LIST, CHAIN_APPROX_NONE);

    // 5) 簡化好輪廓 DP演算法
    vector<vector<Point>> polyContours2(contours2.size()); // 存放折線點的集合
    Mat dp_image_2 = Mat::zeros(dp_optim_v1_image.size(), CV_8UC3);

    for (size_t i = 0; i < contours2.size(); i++)
    {
        approxPolyDP(Mat(contours2[i]), polyContours2[i], 1, true);
    }
    // cout << polyContours2.size();
    drawContours(dp_image_2, polyContours2, -1, Scalar(255, 0, 255), 2, 0);
    Mat dp_image_text = dp_image_2.clone();
    imshow("Contours Image (2):", dp_image_text);
    // drawContours(dp_image_text, polyContours2, 0, Scalar(255, 0, 255), 1, 0);

    // 7) 擬和旋轉矩形 + 邊長數量判斷字型 + 標示方塊中心點
    RotatedRect box;     // 旋轉矩形 class
    Point2f vertices[4]; // 旋轉矩形四頂點
    vector<Point> pt;    // 存一個contour中的點集合
    Point2f centerPoints[10];
    int centerPointNumber = 0;
    int contourNumbers = polyContours2.size();
    // contourNumbers = (contourNumbers > 0) ? 1 : 0;

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
        if (polyContours2[a].size() == 4)
        {
            // rectangleCount++;
            // for (int i = 0; i < 4; i++)
            // {
            //     line(dp_image_2, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 2); // 描出旋轉矩形
            // }
            // 標示
            circle(dp_image_2, (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, 0, Scalar(0, 255, 255), 8);     // 繪製中心點
            circle(original_image, (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, 0, Scalar(0, 255, 255), 8); // 與原圖比較
            centerPoints[centerPointNumber] = (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4;
            centerPointNumber++;
        }
    }
    // for (int i = 0; i < centerPointNumber; i++)
    // {
    //     // cout << centerPoints[j] << endl;
    // }
    if (centerPointNumber >= 4)
    {
        angle = (atan((centerPoints[0].y - centerPoints[centerPointNumber - 1].y) / (centerPoints[centerPointNumber - 1].x - centerPoints[0].x))) / PI * 180;
        if (abs(angle) > 45)
        {
            riseCount++;
            //     cout << "Rise\n";
        }
        else if (abs(angle) < 30)
        {
            downCount++;
            //     cout << "Down\n";
        }
        detectedCounter = (riseCount + downCount);
    }
    // else
    //     angle = 400;
    // cout << angle << "degree\n";
    isRise = (riseCount > downCount) ? true : false;
    // imshow("contour info", contours_info(dp_image_text, polyContours2));

    // imshow("D", dp_image_2);
    // imshow("camera", original_image);
    // cout << "Triangle Count: " << rectangleCount << endl;
    return;
}
void VISION::turnSignImage(Mat original_image, Mat image)
{
    double epsilon = 9;          // DP Algorithm 的參數
    int minContour = 6;          // 邊數小於 minContour 會被遮罩
    int maxContour = 20;         // 邊數大於 maxContour 會遮罩
    double lowerBondArea = 2000; // 面積低於 lowerBondArea 的輪廓會被遮罩

    cvtColor(image, image, COLOR_BGR2GRAY);
    threshold(image, image, 10, 255, THRESH_BINARY);

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
    int leftPoints = 0;
    int rightPoints = 0;
    int contourNumbers = polyContours2.size();
    contourNumbers = (contourNumbers > 0) ? 1 : 0;

    for (int a = 0; a < contourNumbers; a++)
    {
        leftPoints = 0;
        rightPoints = 0;
        // A) 旋轉矩形
        pt.clear();
        for (int b = 0; b < polyContours2[a].size(); b++)
        {
            pt.push_back(polyContours2[a][b]);
        }
        // box = minAreaRect(pt); // 找到最小矩形，存到 box 中
        // box.points(vertices);  // 把矩形的四個頂點資訊丟給 vertices，points()是 RotatedRect 的函式

        // for (int i = 0; i < 4; i++)
        // {
        //     line(dp_image_2, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 2); // 描出旋轉矩形
        // }
        // // 標示
        // circle(dp_image_2, (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, 0, Scalar(0, 255, 255), 8);     // 繪製中心點
        // circle(original_image, (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, 0, Scalar(0, 255, 255), 8); // 與原圖比較

        for (int b = 0; b < polyContours2[a].size(); b++)
        {
            // circle(dp_image_2, polyContours2[a][b], 0, Scalar(0, 255, 255), 4);
            if (polyContours2[a][b].x < ((vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4).x)
                leftPoints++;
            else
                rightPoints++;
        }
        if (leftPoints + rightPoints < 20 && leftPoints + rightPoints > 5)
        {
            if (leftPoints < rightPoints)
                leftCount++;
            else if (leftPoints > rightPoints)
                rightCount++;
            detectedCounter = (leftCount + rightCount);
        }
        if (leftCount > rightCount)
            direction = 'L';
        else if (leftCount < rightCount)
            direction = 'R';
        // putText(original_image, direction, Point(10, 25), 0, 0.8, Scalar(0, 255, 0), 1, 1, false);
    }
    // cout << leftPoints << rightPoints << endl;
    // imshow("D", dp_image_2);
    // imshow("camera", original_image);
    return;
}
void VISION::stop_sign_image()
{
}
void VISION::parking_sign_image()
{
}
Mat VISION::filtGraph(Mat img, int colorCode)
{
    Mat img_hsv, mask, result;
    img_hsv = Mat::zeros(img.size(), CV_8UC3);
    mask = Mat::zeros(img.size(), CV_8UC3);
    result = Mat::zeros(img.size(), CV_8UC3);

    cvtColor(img, img_hsv, COLOR_BGR2HSV);

    // hsv 值改放在 param server
    switch (colorCode)
    {
    case RED1:
    { // green range
        VISION::hue_m = 0;
        VISION::hue_M = 20;
        VISION::sat_m = 120;
        VISION::sat_M = 255;
        VISION::val_m = 0;
        VISION::val_M = 255;
    }
    break;
    case RED2:
    { // white range
        VISION::hue_m = 0;
        VISION::hue_M = 20;
        VISION::sat_m = 120;
        VISION::sat_M = 255;
        VISION::val_m = 0;
        VISION::val_M = 255;
    }
    break;
    case BLUE:
    { // blue range
        VISION::hue_m = 90;
        VISION::hue_M = 140;
        VISION::sat_m = 115;
        VISION::sat_M = 255;
        VISION::val_m = 0;
        VISION::val_M = 200;
    }
    break;
    case GREEN:
    { // blue turn sign(draw) range
        VISION::hue_m = 89;
        VISION::hue_M = 123;
        VISION::sat_m = 132;
        VISION::sat_M = 255;
        VISION::val_m = 10;
        VISION::val_M = 255;
    }
    break;

    default:
        break;
    }

    Scalar lower(hue_m, sat_m, val_m);
    Scalar upper(hue_M, sat_M, val_M);
    inRange(img_hsv, lower, upper, mask);

    bitwise_and(img, img, result, mask);
    // imshow("Letter Filted", result);
    return result;
}
