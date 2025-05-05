#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;

// Global variables
Mat src, src_color, src_hsv, adaptive_threshold;
Mat task1_result, task2_result, task3_result, task4_result;
int blockSize_slider = 3;  // Initial block size
int C_slider = 25;         // Initial C value
int min_area_slider = 100; // Initial minimum area for filtering
int kernel_size_slider = 3; // Initial kernel size for morphological operations

// HSV filtering parameters
int h_min = 15, s_min = 50, v_min = 50;
int h_max = 30, s_max = 255, v_max = 255;

// Function declarations
void processTask1();
void processTask2();
void processTask3();
void processTask4();

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
        pos,
        C_slider
    );
    
    // Display results
    imshow("THRESHOLD", adaptive_threshold);
    
    // Process all tasks
    processTask1();
    processTask2();
    processTask3();
    processTask4();
    
    // Print current values to console
    std::cout << "Block Size: " << pos << ", C: " << C_slider << std::endl;
}

// Callback function for C value trackbar
static void on_C_trackbar(int, void*) {
    on_blockSize_trackbar(getTrackbarPos("Block Size", "Controls"), nullptr);
}

// Callback function for minimum area trackbar
static void on_min_area_trackbar(int, void*) {
    processTask3();
}

// Callback function for kernel size trackbar
static void on_kernel_size_trackbar(int pos, void*) {
    // Force position to be odd
    if (pos < 3) pos = 3;
    
    if (pos % 2 == 0) {
        pos = pos + 1;
        setTrackbarPos("Kernel Size", "Controls", pos);
        return;
    }
    
    processTask2();
}

// Callbacks for HSV trackbars
static void on_hsv_trackbar(int, void*) {
    processTask4();
}

// Task 1: Basic Object Localization
void processTask1() {
    task1_result = src_color.clone();
    
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    Mat binary_copy = adaptive_threshold.clone();
    findContours(binary_copy, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    RNG rng(12345);
    for (size_t i = 0; i < contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        drawContours(task1_result, contours, static_cast<int>(i), color, 2);
        Rect bounding = boundingRect(contours[i]);
        rectangle(task1_result, bounding, color, 2);
        circle(task1_result, Point(bounding.x + bounding.width/2, 
               bounding.y + bounding.height/2), 5, color, -1);
    }
    
    imshow("Task 1: Basic Contour Detection", task1_result);
}

// Task 2: Morphological Opening
void processTask2() {
    task2_result = src_color.clone();
    
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(kernel_size_slider, kernel_size_slider));
    
    Mat opened;
    morphologyEx(adaptive_threshold, opened, MORPH_OPEN, kernel);
    
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(opened, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    RNG rng(12345);
    for (size_t i = 0; i < contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        drawContours(task2_result, contours, static_cast<int>(i), color, 2);
        Rect bounding = boundingRect(contours[i]);
        rectangle(task2_result, bounding, color, 2);
        circle(task2_result, Point(bounding.x + bounding.width/2, 
               bounding.y + bounding.height/2), 5, color, -1);
    }
    
    imshow("Morphological Opening", opened);
    imshow("Task 2: Morphological Opening", task2_result);
}

// Task 3: Area-based Filtering
void processTask3() {
    task3_result = src_color.clone();
    
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    Mat binary_copy = adaptive_threshold.clone();
    findContours(binary_copy, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    RNG rng(12345);
    for (size_t i = 0; i < contours.size(); i++) {
        if (contourArea(contours[i]) > min_area_slider) {
            Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
            drawContours(task3_result, contours, static_cast<int>(i), color, 2);
            Rect bounding = boundingRect(contours[i]);
            rectangle(task3_result, bounding, color, 2);
            circle(task3_result, Point(bounding.x + bounding.width/2, 
                   bounding.y + bounding.height/2), 5, color, -1);
        }
    }
    
    imshow("Task 3: Area-based Filtering", task3_result);
}

// Task 4: HSV Color Filtering
void processTask4() {
    task4_result = src_color.clone();
    
    Mat hsv_mask;
    inRange(src_hsv, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), hsv_mask);
    
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(hsv_mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    RNG rng(12345);
    for (size_t i = 0; i < contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        drawContours(task4_result, contours, static_cast<int>(i), color, 2);
        Rect bounding = boundingRect(contours[i]);
        rectangle(task4_result, bounding, color, 2);
        circle(task4_result, Point(bounding.x + bounding.width/2, 
               bounding.y + bounding.height/2), 5, color, -1);
    }
    
    imshow("HSV Mask", hsv_mask);
    imshow("Task 4: HSV Color Filtering", task4_result);
}

int main() {
    // Load image (both color and grayscale)
    src_color = imread("../data/1.png", IMREAD_COLOR);
    
    if (src_color.empty()) {
        std::cout << "Could not open or find the image!\n";
        return -1;
    }
    
    // Convert to grayscale for thresholding
    cvtColor(src_color, src, COLOR_BGR2GRAY);
    
    // Convert to HSV for color filtering (Task 4)
    cvtColor(src_color, src_hsv, COLOR_BGR2HSV);
    
    // Create windows
    namedWindow("Original Image", WINDOW_AUTOSIZE);
    namedWindow("THRESHOLD", WINDOW_AUTOSIZE);
    namedWindow("Controls", WINDOW_AUTOSIZE);
    namedWindow("Task 1: Basic Contour Detection", WINDOW_AUTOSIZE);
    namedWindow("Task 2: Morphological Opening", WINDOW_AUTOSIZE);
    namedWindow("Morphological Opening", WINDOW_AUTOSIZE);
    namedWindow("Task 3: Area-based Filtering", WINDOW_AUTOSIZE);
    namedWindow("HSV Mask", WINDOW_AUTOSIZE);
    namedWindow("Task 4: HSV Color Filtering", WINDOW_AUTOSIZE);
    namedWindow("HSV Controls", WINDOW_AUTOSIZE);
    
    // Create trackbars for adaptive threshold
    createTrackbar("Block Size", "Controls", &blockSize_slider, 51, on_blockSize_trackbar);
    createTrackbar("C Value", "Controls", &C_slider, 100, on_C_trackbar);
    
    // Create trackbar for minimum area (Task 3)
    createTrackbar("Min Area", "Controls", &min_area_slider, 1000, on_min_area_trackbar);
    
    // Create trackbar for kernel size (Task 2)
    createTrackbar("Kernel Size", "Controls", &kernel_size_slider, 21, on_kernel_size_trackbar);
    
    // Create trackbars for HSV filtering (Task 4)
    createTrackbar("H min", "HSV Controls", &h_min, 179, on_hsv_trackbar);
    createTrackbar("S min", "HSV Controls", &s_min, 255, on_hsv_trackbar);
    createTrackbar("V min", "HSV Controls", &v_min, 255, on_hsv_trackbar);
    createTrackbar("H max", "HSV Controls", &h_max, 179, on_hsv_trackbar);
    createTrackbar("S max", "HSV Controls", &s_max, 255, on_hsv_trackbar);
    createTrackbar("V max", "HSV Controls", &v_max, 255, on_hsv_trackbar);
    
    // Set initial positions
    setTrackbarPos("Block Size", "Controls", 3);
    setTrackbarPos("Kernel Size", "Controls", 3);
    
    // Initialize HSV trackbars to default yellow values
    setTrackbarPos("H min", "HSV Controls", 15);
    setTrackbarPos("H max", "HSV Controls", 30);
    setTrackbarPos("S min", "HSV Controls", 50);
    setTrackbarPos("S max", "HSV Controls", 255);
    setTrackbarPos("V min", "HSV Controls", 50);
    setTrackbarPos("V max", "HSV Controls", 255);
    
    // Display original image
    imshow("Original Image", src_color);
    
    // Initialize threshold image and all tasks
    on_blockSize_trackbar(3, nullptr);
    
    // Loop until a key is pressed
    while (true) {
        int key = waitKey(1);
        if (key == 'q' || key == 27) { // Exit on 'q' or ESC key
            break;
        }   
    }
    
    destroyAllWindows();
    return 0;
}
