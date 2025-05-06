#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// --- Trackbar parameters ---
int cannyThreshold1 = 150;
int cannyThreshold2 = 500;
int houghThreshold = 20;
int minLineLength = 40;
int maxLineGap = 1;

Mat src;

// --- Processing function called by trackbars and main loop ---
void processAndShow() {
    Mat edges, color;
    Canny(src, edges, cannyThreshold1, cannyThreshold2, 3);
    cvtColor(edges, color, COLOR_GRAY2BGR);

    vector<Vec4i> linesP;
    HoughLinesP(edges, linesP, 1, CV_PI / 180, houghThreshold, minLineLength, maxLineGap);

    for (const auto& l : linesP) {
        line(color, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, LINE_AA);
    }

    string text = "Lines: " + to_string(linesP.size());
    putText(color, text, Point(20, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);

    imshow("Original", src);
    imshow("Canny", edges);
    imshow("Lines", color);
}

// --- Trackbar callback: triggers processing ---
static void onTrackbar(int, void*) {
    processAndShow();
}

int main() {
    src = imread("../data/road.jpg", 1);
    if (src.empty()) {
        cout << "Could not open or find the image!" << endl;
        return -1;
    }

    // --- Create windows and trackbars for interactive parameter tuning ---
    namedWindow("Lines", WINDOW_AUTOSIZE);
    namedWindow("Canny", WINDOW_AUTOSIZE);
    namedWindow("Original", WINDOW_AUTOSIZE);
    namedWindow("Controls", WINDOW_AUTOSIZE);

    createTrackbar("Canny Threshold1", "Controls", &cannyThreshold1, 500, onTrackbar);
    createTrackbar("Canny Threshold2", "Controls", &cannyThreshold2, 500, onTrackbar);
    createTrackbar("Hough Threshold", "Controls", &houghThreshold, 100, onTrackbar);
    createTrackbar("Min Line Length", "Controls", &minLineLength, 200, onTrackbar);
    createTrackbar("Max Line Gap", "Controls", &maxLineGap, 50, onTrackbar);

    processAndShow(); // Initial processing

    // --- Main loop: wait for user to exit ---
    while (true) {
        int key = waitKey(1);
        if (key == 'q' || key == 27) break;
    }

    destroyAllWindows();
    return 0;
}
