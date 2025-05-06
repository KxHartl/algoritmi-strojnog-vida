#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;

// --- Global variables for image and trackbar parameters ---
Mat src, adaptive_threshold;
int blockSize_slider = 3; 
int C_slider = 25;      

// --- Trackbar callback for block size (adaptive threshold parameter) ---
static void on_blockSize_trackbar(int pos, void*) {
    // Ensure block size is odd and >= 3
    if (pos < 3) pos = 3;  
    if (pos % 2 == 0) {    
        pos += 1;
        setTrackbarPos("Block Size", "Controls", pos);
        return; 
    }

    // Apply adaptive thresholding with current parameters
    adaptiveThreshold(src, adaptive_threshold, 255,
        ADAPTIVE_THRESH_GAUSSIAN_C,
        THRESH_BINARY_INV,
        pos,
        C_slider
    );
    imshow("THRESHOLD", adaptive_threshold);

    std::cout << "Block Size: " << pos << ", C: " << C_slider << std::endl;
}

// --- Trackbar callback for C value (adaptive threshold parameter) ---
static void on_C_trackbar(int, void*) {
    on_blockSize_trackbar(getTrackbarPos("Block Size", "Controls"), nullptr);
}

int main() {
    // --- Load grayscale image ---
    src = imread("../data/parts.png", IMREAD_GRAYSCALE);
    if (src.empty()) {
        std::cout << "Could not open or find the image!\n";
        return -1;
    }

    // --- Create windows and trackbars for interactive parameter tuning ---
    namedWindow("Original Image", WINDOW_AUTOSIZE);
    namedWindow("THRESHOLD", WINDOW_AUTOSIZE);
    namedWindow("Controls", WINDOW_AUTOSIZE);

    createTrackbar("Block Size", "Controls", &blockSize_slider, 201, on_blockSize_trackbar);
    createTrackbar("C Value", "Controls", &C_slider, 51, on_C_trackbar);

    setTrackbarPos("Block Size", "Controls", 3);

    imshow("Original Image", src);

    // --- Initial thresholding and display ---
    on_blockSize_trackbar(3, nullptr);

    // --- Main loop: wait for user to exit ---
    while (true) {
        int key = waitKey(1);
        if (key == 'q' || key == 27) break;
    }

    destroyAllWindows();
    return 0;
}
