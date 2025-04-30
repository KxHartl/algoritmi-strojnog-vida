#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(){
    Mat image=imread("1.png",1);
    Mat image_gray=imread("1.png",0);
    Mat image_binary,image_hsv;
    //threshold(image_gray,image_binary,100,255,THRESH_BINARY_INV+THRESH_OTSU);
    //threshold(image_gray,image_binary,100,255,THRESH_BINARY_INV);
    adaptiveThreshold(image_gray, image_binary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 131, 5);
    vector<vector<Point> > contours;
    findContours(image_binary,contours,RETR_EXTERNAL,1);
    RNG rng(12345); //range for random number generation
    for (size_t i = 0; i < contours.size(); i++)//for each contour
    {
    if (contourArea(contours.at(i))>100) //filter for removing small contours
    {
    Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256)); //generate random color
    drawContours(image, contours, (int)i, color, 2, LINE_8);//draw contour "i"
    Rect bounding = boundingRect(contours.at(i)); //creates bounding rectangle around contour "i"
    // approximate contour with accuracy proportional to the contour perimeter
    }
    }
    imshow("Color", image);
    imshow("Gray", image_gray);
    imshow("Binary", image_binary);
    waitKey(0);
    return 1;
    }