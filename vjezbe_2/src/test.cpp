#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
// Print OpenCV version
// std::cout << "OpenCV version: " << CV_VERSION <<
// std::endl;
// // Create a simple image
// cv::Mat image = cv::Mat::zeros(300, 300, CV_8UC3);
// // Draw a red circle
// cv::circle(image, cv::Point(150, 150), 50, cv::Scalar(0, 0, 255), -1);

// Mat image = imread("data/screws.jpg", 0);
// Rect myROI(50, 50, 100, 100);
// Mat image_ROI = image(myROI).clone();

// Mat image2 = image.clone();
// // cvtColor(image, image2, COLOR_RGB2GRAY);
// // resize(image2, image2, Size(), 0.5, 0.5, INTER_LINEAR);
// image2.convertTo(image, -1, 1.5, 0.5);

Mat image = Mat::zeros(Size(200, 200), CV_8U);
circle(image, Point(100,100),30,Scalar(255),-1,LINE_8,0);
rectangle(image, Rect(20,20,160,160),Scalar(255),10,LINE_8,0);
line(image,Point(0,100),Point(200,100),Scalar(0),10,LINE_8,0);

cout<<image.empty()<<endl;
cout<<image.size()<<endl;
cout<<image.channels()<<endl;

// Show the image
imshow("OpenCV Test", image);
// imshow("OpenCV Test2", image2);
// imshow("OpenCV Test3", image_ROI);

waitKey(0); // Wait for key press
return 0;
}