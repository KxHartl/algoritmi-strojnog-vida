#include <opencv2/opencv.hpp>
#include <iostream>

// Global variables
cv::Mat image, filteredImage;
int kernelSize = 3; // Default kernel size for filters

// Callback function for the trackbar
void applyFilters(int, void*) {
    // Apply bilateral filter
    cv::bilateralFilter(image, filteredImage, kernelSize, kernelSize * 2, kernelSize / 2);
    
    // Display the filtered image
    cv::imshow("Filtered Image", filteredImage);
}

int main() {
    // Load the input image
    image = cv::imread("data/parts.png", cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "Error: Could not load image!" << std::endl;
        return -1;
    }

    // Convert to grayscale if needed
    if (image.channels() == 3) {
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    }

    // Create a window
    cv::namedWindow("Filtered Image", cv::WINDOW_AUTOSIZE);

    // Create a trackbar to adjust kernel size
    cv::createTrackbar("Kernel Size", "Filtered Image", &kernelSize, 15, applyFilters);

    // Apply filters initially
    applyFilters(0, 0);

    // Wait for user interaction
    cv::waitKey(0);

    // Morphological Hit-or-Miss to find corners of a rectangle
    cv::Mat hitMissResult;
    
    // Define kernels for hit-or-miss (example for detecting corners)
    cv::Mat kernel1 = (cv::Mat_<int>(3, 3) << -1, -1, -1,
                                              -1,  1, -1,
                                              -1, -1, -1);
                                              
    cv::Mat kernel2 = (cv::Mat_<int>(3, 3) << -1,  0, -1,
                                               0,  1,  0,
                                              -1,  0, -1);

    // Apply hit-or-miss operation
    cv::morphologyEx(image, hitMissResult, cv::MORPH_HITMISS, kernel1);
    
    // Display the hit-or-miss result
    cv::imshow("Hit-or-Miss Result", hitMissResult);

    // Wait for user interaction before closing
    cv::waitKey(0);

    return 0;
}
