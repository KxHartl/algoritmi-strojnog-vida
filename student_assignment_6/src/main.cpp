#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

// Helper function to resize images for display
Mat resizeForDisplay(const Mat& img, double scale) {
    Mat resized;
    resize(img, resized, Size(), scale, scale, INTER_AREA);
    return resized;
}

int main() {
    const double DISPLAY_SCALE = 0.25;
    
    // Load input image
    Mat inputImage = imread("../data/IMG_3397.jpg", 1);
    if (inputImage.empty()) {
        cerr << "Could not open or find the image!" << endl;
        return -1;
    }

    Mat visualizationImage = inputImage.clone();

    // --- ArUco Marker Detection Setup ---
    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_250);
    aruco::DetectorParameters parameters;
    parameters.adaptiveThreshWinSizeMin = 23;
    parameters.adaptiveThreshWinSizeMax = 53;
    parameters.adaptiveThreshWinSizeStep = 10;
    
    aruco::ArucoDetector detector(dictionary, parameters);
    vector<vector<Point2f>> markerCorners;
    vector<int> markerIds;
    detector.detectMarkers(inputImage, markerCorners, markerIds);
    
    // Ensure at least 4 markers are detected for homography
    if (markerCorners.size() < 4) {
        cerr << "Not enough markers detected. Found: " << markerCorners.size() << ", need at least 4." << endl;
        return -1;
    }

    // Visualize detected markers
    aruco::drawDetectedMarkers(visualizationImage, markerCorners, markerIds);
    imshow("Detected Markers", resizeForDisplay(visualizationImage, DISPLAY_SCALE));

    // --- Calculate Centers of Each Marker ---
    vector<Point2f> markerCenters;
    for (const auto& corners : markerCorners) {
        Point2f center(0, 0);
        for (const auto& corner : corners) {
            center += corner;
        }
        center *= 1.0f / 4.0f;
        markerCenters.push_back(center);
    }
    
    // --- Identify Corners for Perspective Transform ---
    // Find indices for top-left, top-right, bottom-right, bottom-left markers
    int topLeftIdx = -1, topRightIdx = -1, bottomRightIdx = -1, bottomLeftIdx = -1;
    float minSum = FLT_MAX, maxSum = -FLT_MAX;
    float minDiff = FLT_MAX, maxDiff = -FLT_MAX;
    
    for (size_t i = 0; i < markerCenters.size(); i++) {
        float sum = markerCenters[i].x + markerCenters[i].y;
        float diff = markerCenters[i].x - markerCenters[i].y;
        
        if (sum < minSum) { minSum = sum; topLeftIdx = i; }
        if (sum > maxSum) { maxSum = sum; bottomRightIdx = i; }
        if (diff > maxDiff) { maxDiff = diff; topRightIdx = i; }
        if (diff < minDiff) { minDiff = diff; bottomLeftIdx = i; }
    }
    
    // Source and destination points for homography
    vector<Point2f> sourcePoints = {
        markerCenters[topLeftIdx],
        markerCenters[topRightIdx],
        markerCenters[bottomRightIdx],
        markerCenters[bottomLeftIdx]
    };
    
    // --- Compute Output Size for Rectification ---
    float width1 = norm(sourcePoints[1] - sourcePoints[0]);
    float width2 = norm(sourcePoints[2] - sourcePoints[3]);
    float height1 = norm(sourcePoints[3] - sourcePoints[0]);
    float height2 = norm(sourcePoints[2] - sourcePoints[1]);
    float avgWidth = (width1 + width2) / 2.0f;
    float avgHeight = (height1 + height2) / 2.0f;
    
    vector<Point2f> destinationPoints = {
        Point2f(0, 0),
        Point2f(avgWidth, 0),
        Point2f(avgWidth, avgHeight),
        Point2f(0, avgHeight)
    };
    
    // --- Perspective Rectification ---
    Mat homography = findHomography(sourcePoints, destinationPoints);
    Mat rectifiedImage;
    warpPerspective(inputImage, rectifiedImage, homography, Size(avgWidth, avgHeight));
    imshow("Rectified Image", resizeForDisplay(rectifiedImage, DISPLAY_SCALE));

    // --- Thresholding for Contour Detection ---
    Mat grayImage, binaryImage;
    cvtColor(rectifiedImage, grayImage, COLOR_BGR2GRAY);
    adaptiveThreshold(grayImage, binaryImage, 255,
        ADAPTIVE_THRESH_GAUSSIAN_C, 
        THRESH_BINARY_INV,
        119,
        9);

    imshow("Thresholded Image", resizeForDisplay(binaryImage, DISPLAY_SCALE));
    
    // --- Contour Detection and Visualization ---
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binaryImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    Mat result = rectifiedImage.clone();
    
    for (size_t i = 0; i < contours.size(); i++) {
        Scalar color = Scalar(0, 0, 0);
        vector<Point> approx;
        double epsilon = 0.02 * arcLength(contours[i], true);
        approxPolyDP(contours[i], approx, epsilon, true);
        
        drawContours(result, contours, static_cast<int>(i), color, 6);
        // Calculate centroid of each contour (not used further)
        Moments m = moments(contours[i]);
        int cx = static_cast<int>(m.m10 / m.m00);
        int cy = static_cast<int>(m.m01 / m.m00);
    }

    imshow("Result", resizeForDisplay(result, DISPLAY_SCALE));

    // --- Wait for User to Exit ---
    while (true) {
        int key = waitKey(1);
        if (key == 'q' || key == 27) { // Exit on 'q' or ESC key
            break;
        }   
    }
    
    destroyAllWindows();
    return 0;
}
