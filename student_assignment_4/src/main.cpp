#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;

// Global variables
Mat src, adaptive_threshold;
int blockSize_slider = 3;  // Initial block size
int C_slider = 25;         // Initial C value

// Callback function for blockSize trackbar
static void on_blockSize_trackbar(int pos, void*) {
    // Force position to be odd (3, 5, 7, ...)
    if (pos < 3) pos = 3;  // Minimum block size is 3
    
    if (pos % 2 == 0) {    // If even, make it odd
        pos = pos + 1;
        setTrackbarPos("Block Size", "Controls", pos);
        return;  // Will be called again with the new position
    }
    
    // Apply adaptive thresholding with current parameters
    adaptiveThreshold(
        src,
        adaptive_threshold,
        255,
        ADAPTIVE_THRESH_GAUSSIAN_C,
        THRESH_BINARY_INV,
        pos,  // Block size is directly the position (which is now odd)
        C_slider
    );
    
    // Display results
    imshow("THRESHOLD", adaptive_threshold);
    
    // Print current values to console
    std::cout << "Block Size: " << pos << ", C: " << C_slider << std::endl;
}

// Callback function for C value trackbar
static void on_C_trackbar(int, void*) {
    // Call the blockSize trackbar callback to update the threshold
    on_blockSize_trackbar(getTrackbarPos("Block Size", "Controls"), nullptr);
}

int main() {
    // Load image in grayscale
    src = imread("../data/parts.png", IMREAD_GRAYSCALE);
    
    // Check if image is loaded
    if (src.empty()) {
        std::cout << "Could not open or find the image!\n";
        return -1;
    }
    
    // Create windows
    namedWindow("Original Image", WINDOW_AUTOSIZE);
    namedWindow("THRESHOLD", WINDOW_AUTOSIZE);
    namedWindow("Controls", WINDOW_AUTOSIZE);
    
    // Create trackbars
    createTrackbar("Block Size", "Controls", &blockSize_slider, 51, on_blockSize_trackbar);
    createTrackbar("C Value", "Controls", &C_slider, 100, on_C_trackbar);
    
    // Set initial block size to 3
    setTrackbarPos("Block Size", "Controls", 3);
    
    // Display original image
    imshow("Original Image", src);
    
    // Initialize threshold image
    on_blockSize_trackbar(3, nullptr);
    
    // Wait for a key press
    waitKey(0);
    
    // Clean up
    destroyAllWindows();
    return 0;
}
