#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
// Print OpenCV version
std::cout << "OpenCV version: " << CV_VERSION <<
std::endl;
// Create a simple image
cv::Mat image = cv::Mat::zeros(300, 300, CV_8UC3);
// Draw a red circle
cv::circle(image, cv::Point(150, 150), 50, cv::Scalar(0, 0,
255), -1);
// Show the image
cv::imshow("OpenCV Test", image);
cv::waitKey(0); // Wait for key press
return 0;
}