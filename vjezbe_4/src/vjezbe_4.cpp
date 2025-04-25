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
    image = cv::imread("data/gray_text.png", cv::IMREAD_COLOR);
    // Convert to grayscale if needed
    if (image.channels() == 3) {
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    }

    // Create a window
    cv::namedWindow("Filtered Image", cv::WINDOW_AUTOSIZE);

    cv::waitKey(0); // Wait for key press
    return 0;
}
