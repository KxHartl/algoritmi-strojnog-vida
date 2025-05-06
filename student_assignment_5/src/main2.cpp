#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// --- Global variables for images and UI parameters ---
Mat src, src_color, src_hsv, adaptive_threshold;
Mat task1_result, task2_result, task3_result, task4_result;
int blockSize_slider = 3;
int C_slider = 25;      
int min_area_slider = 100;
int kernel_size_slider = 3;

// HSV filtering parameters
int h_min = 15, s_min = 50, v_min = 50;
int h_max = 30, s_max = 255, v_max = 255;

// Forward declarations for processing functions
void processTask1();
void processTask2();
void processTask3();
void processTask4();

// --- Trackbar callback: Adaptive threshold block size ---
static void on_blockSize_trackbar(int pos, void*) {
    // Ensure block size is odd and >= 3
    if (pos < 3) pos = 3;
    if (pos % 2 == 0) {
        pos = pos + 1;
        setTrackbarPos("Block Size", "Controls", pos);
        return;
    }
    
    // Update adaptive threshold image
    adaptiveThreshold(src, adaptive_threshold, 255,
        ADAPTIVE_THRESH_GAUSSIAN_C,
        THRESH_BINARY_INV,
        pos,
        C_slider
    );
    imshow("THRESHOLD", adaptive_threshold);

    // Re-run all processing tasks on threshold change
    processTask1();
    processTask2();
    processTask3();
    processTask4();
    
    cout << "Block Size: " << pos << ", C: " << C_slider << endl;
}

// --- Trackbar callback: Adaptive threshold C value ---
static void on_C_trackbar(int, void*) {
    on_blockSize_trackbar(getTrackbarPos("Block Size", "Controls"), nullptr);
}

// --- Trackbar callback: Minimum area for filtering ---
static void on_min_area_trackbar(int, void*) {
    processTask3();
}

// --- Trackbar callback: Morphology kernel size ---
static void on_kernel_size_trackbar(int pos, void*) {
    // Ensure kernel size is odd and >= 3
    if (pos < 3) pos = 3;
    if (pos % 2 == 0) {
        pos = pos + 1;
        setTrackbarPos("Kernel Size", "Controls", pos);
        return;
    }
    processTask2();
}

// --- Trackbar callback: HSV filter changes ---
static void on_hsv_trackbar(int, void*) {
    processTask4();
}

// --- Task 1: Basic Contour Detection on thresholded image ---
void processTask1() {
    task1_result = src_color.clone();
    
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Mat binary_copy = adaptive_threshold.clone();
    findContours(binary_copy, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    RNG rng(12345);
    for (size_t i = 0; i < contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        drawContours(task1_result, contours, static_cast<int>(i), color, 2);
        Rect bounding = boundingRect(contours[i]);
        rectangle(task1_result, bounding, color, 2);
        // Draw center of bounding box
        circle(task1_result, Point(bounding.x + bounding.width/2, 
               bounding.y + bounding.height/2), 5, color, -1);
    }
    
    imshow("Task 1: Basic Contour Detection", task1_result);
}

// --- Task 2: Contour Detection after Morphological Opening ---
void processTask2() {
    task2_result = src_color.clone();
    // Morphological opening to remove noise
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(kernel_size_slider, kernel_size_slider));
    Mat opened;
    morphologyEx(adaptive_threshold, opened, MORPH_OPEN, kernel);
    
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
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

// --- Task 3: Area-based Contour Filtering ---
void processTask3() {
    task3_result = src_color.clone();
    
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Mat binary_copy = adaptive_threshold.clone();
    findContours(binary_copy, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    RNG rng(12345);
    for (size_t i = 0; i < contours.size(); i++) {
        // Only draw contours with area above threshold
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

// --- Task 4: HSV-based Color Filtering and Contour Detection ---
void processTask4() {
    task4_result = src_color.clone();
    // HSV color thresholding
    Mat hsv_mask;
    inRange(src_hsv, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), hsv_mask);
    
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
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
    // --- Load and prepare images ---
    src_color = imread("../data/1.png", IMREAD_COLOR);
    if (src_color.empty()) {
        cout << "Could not open or find the image!\n";
        return -1;
    }
    cvtColor(src_color, src, COLOR_BGR2GRAY);
    cvtColor(src_color, src_hsv, COLOR_BGR2HSV);

    // --- Create UI windows and trackbars for interactive parameter tuning ---
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

    // --- Controls for adaptive thresholding and morphology ---
    createTrackbar("Block Size", "Controls", &blockSize_slider, 151, on_blockSize_trackbar);
    createTrackbar("C Value", "Controls", &C_slider, 100, on_C_trackbar);
    createTrackbar("Min Area", "Controls", &min_area_slider, 1000, on_min_area_trackbar);
    createTrackbar("Kernel Size", "Controls", &kernel_size_slider, 21, on_kernel_size_trackbar);

    // --- Controls for HSV color filtering ---
    createTrackbar("H min", "HSV Controls", &h_min, 179, on_hsv_trackbar);
    createTrackbar("S min", "HSV Controls", &s_min, 255, on_hsv_trackbar);
    createTrackbar("V min", "HSV Controls", &v_min, 255, on_hsv_trackbar);
    createTrackbar("H max", "HSV Controls", &h_max, 179, on_hsv_trackbar);
    createTrackbar("S max", "HSV Controls", &s_max, 255, on_hsv_trackbar);
    createTrackbar("V max", "HSV Controls", &v_max, 255, on_hsv_trackbar);

    // --- Set initial trackbar positions ---
    setTrackbarPos("Block Size", "Controls", 3);
    setTrackbarPos("Kernel Size", "Controls", 3);
    setTrackbarPos("H min", "HSV Controls", 15);
    setTrackbarPos("H max", "HSV Controls", 30);
    setTrackbarPos("S min", "HSV Controls", 50);
    setTrackbarPos("S max", "HSV Controls", 255);
    setTrackbarPos("V min", "HSV Controls", 50);
    setTrackbarPos("V max", "HSV Controls", 255);

    imshow("Original Image", src_color);

    // --- Initial processing and display ---
    on_blockSize_trackbar(3, nullptr);

    // --- Main loop: Wait for exit key ---
    while (true) {
        int key = waitKey(1);
        if (key == 'q' || key == 27) { // Exit on 'q' or ESC key
            break;
        }   
    }
    
    destroyAllWindows();
    return 0;
}
