#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // Load image
    Mat src = imread("../data/1.png", IMREAD_COLOR);
    Mat result;

    // Convert to grayscale
    cvtColor(src, result, COLOR_BGR2HSV);

    // adaptiveThreshold(result, result, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 121, 2);

    inRange(result, Scalar(15, 50, 50), Scalar(30, 250, 250), result);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    // morphologyEx(result, result, MORPH_OPEN, element);
    // morphologyEx(result, result, MORPH_DILATE, element);

    std::vector <std::vector<Point>> contours;
    findContours(result, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    RNG rng(12345); //range for random number generation
    for (size_t i = 0; i < contours.size(); i++)    //for each contour
        {
            if (contourArea(contours.at(i))>100){
                
            Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));               //generate random color
            drawContours(src, contours, (int)i, color, 2, LINE_8);                                              //draw contour "i"
            Rect bounding = boundingRect(contours.at(i));                                                       //creates bounding rectangle around contour "i"
            rectangle(src,bounding,color,2,LINE_8,0);                                                           //draw bounding rectangle
            circle(src,Point(bounding.x+bounding.width/2,bounding.y+bounding.height/2),5,color,-1,LINE_8,0);    // draw center of bounding rectange

            }
        
        }

    imshow("Conturs", src);
    imshow("results", result);

    waitKey(0); // Wait for key press
    return 0;
}