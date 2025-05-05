#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// Global variables for trackbars
int cannyThreshold1 = 150;
int cannyThreshold2 = 500;
int houghThreshold = 20;
int minLineLength = 40;
int maxLineGap = 1;

// Original image
Mat src;

// Callback function for trackbars
static void onTrackbar(int, void*) {
    // Will be called when trackbar position changes
    // We process everything in the main loop
}

int main() {
    // Load the image
    src = imread("../data/road.jpg", 0);
    if (src.empty()) {
        cout << "Could not open or find the image!" << endl;
        return -1;
    }

    // Create windows for displaying results and trackbars
    namedWindow("Canny", WINDOW_AUTOSIZE);
    namedWindow("Color", WINDOW_AUTOSIZE);
    namedWindow("Controls", WINDOW_AUTOSIZE);

    // Create trackbars
    createTrackbar("Canny Threshold1", "Controls", &cannyThreshold1, 500, onTrackbar);
    createTrackbar("Canny Threshold2", "Controls", &cannyThreshold2, 500, onTrackbar);
    createTrackbar("Hough Threshold", "Controls", &houghThreshold, 100, onTrackbar);
    createTrackbar("Min Line Length", "Controls", &minLineLength, 200, onTrackbar);
    createTrackbar("Max Line Gap", "Controls", &maxLineGap, 50, onTrackbar);

    // Process loop
    while (true) {
        // Create working copies
        Mat edges = src.clone();
        Mat color;

        // Apply Canny edge detection with trackbar values
        Canny(edges, edges, cannyThreshold1, cannyThreshold2, 3);
        
        // Convert to color for drawing
        cvtColor(edges, color, COLOR_GRAY2BGR);

        // Detect lines using HoughLinesP with trackbar values
        vector<Vec4i> linesP;
        HoughLinesP(edges, linesP, 1, CV_PI/180, houghThreshold, minLineLength, maxLineGap);
        
        // Draw the lines
        for (size_t i = 0; i < linesP.size(); i++) {
            Vec4i l = linesP[i];
            line(color, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, LINE_AA);
        }

        // Add text showing current parameters and line count
        string text = "Lines: " + to_string(linesP.size());
        putText(color, text, Point(20, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);

        // Display the results
        imshow("Original", src);
        imshow("Color", color);
        imshow("Canny", edges);

        // Exit on 'q' or ESC key
        int key = waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
    }

    destroyAllWindows();
    return 1;
}
