#include <opencv2/opencv.hpp>
#include <iostream>

// Global variables
cv::Mat image, gray;
int blockSize = 25;
int cValue = 25;
const char* windowName = "Adaptive Threshold";

// Callback function for trackbar events
void updateThreshold(int, void*)
{
    // Ensure block size is odd (required by adaptiveThreshold)
    if (blockSize % 2 == 0)
        blockSize++;
    
    // Minimum block size is 3
    if (blockSize < 3)
        blockSize = 3;
    
    // Apply adaptive thresholding with current parameters
    cv::Mat result;
    cv::adaptiveThreshold(gray, result, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, blockSize, cValue);
    
    // Display result
    cv::imshow(windowName, result);
}

int main()
{
    // Load image
    image = cv::imread("../data/parts.png");
    if (image.empty()) {
        std::cerr << "Error: Could not open or find the image" << std::endl;
        return -1;
    }
    
    // Convert to grayscale
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    
    // Create window and trackbars
    cv::namedWindow(windowName);
    cv::createTrackbar("Block Size", windowName, &blockSize, 201, updateThreshold);
    cv::createTrackbar("C Value", windowName, &cValue, 50, updateThreshold);
    
    // Initialize display
    updateThreshold(0, 0);
    
    cv::waitKey(0);
    return 0;
}
