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
    // Display scale factor
    const double DISPLAY_SCALE = 0.25;
    
    // Step 1: Load the image
    Mat inputImage = imread("../data/IMG_3397.jpg", 1);
    if (inputImage.empty()) {
        cerr << "Could not open or find the image!" << endl;
        return -1;
    }

    // Create a copy for visualization
    Mat visualizationImage = inputImage.clone();

    // Step 2: Detect ArUco markers
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    cv::aruco::DetectorParameters parameters;
    
    // Adjust parameters for better detection
    parameters.adaptiveThreshWinSizeMin = 23;
    parameters.adaptiveThreshWinSizeMax = 53;
    parameters.adaptiveThreshWinSizeStep = 10;
    
    // Create detector and detect markers
    cv::aruco::ArucoDetector detector(dictionary, parameters);
    vector<vector<Point2f>> markerCorners;
    vector<int> markerIds;
    detector.detectMarkers(inputImage, markerCorners, markerIds);
    
    // Check if we found at least 4 markers
    if (markerCorners.size() < 4) {
        cerr << "Not enough markers detected. Found: " << markerCorners.size() << ", need at least 4." << endl;
        return -1;
    }

    // Draw detected markers on visualization image
    cv::aruco::drawDetectedMarkers(visualizationImage, markerCorners, markerIds);
    imshow("Detected Markers", resizeForDisplay(visualizationImage, DISPLAY_SCALE));

    // Step 3: Calculate marker centers
    vector<Point2f> markerCenters;
    for (const auto& corners : markerCorners) {
        Point2f center(0, 0);
        for (const auto& corner : corners) {
            center += corner;
        }
        center *= 1.0f / 4.0f;
        markerCenters.push_back(center);
    }
    
    // Find the corner markers using sum and difference of coordinates
    int topLeftIdx = -1, topRightIdx = -1, bottomRightIdx = -1, bottomLeftIdx = -1;
    float minSum = FLT_MAX, maxSum = -FLT_MAX;
    float minDiff = FLT_MAX, maxDiff = -FLT_MAX;
    
    for (size_t i = 0; i < markerCenters.size(); i++) {
        float sum = markerCenters[i].x + markerCenters[i].y;
        float diff = markerCenters[i].x - markerCenters[i].y;
        
        if (sum < minSum) {
            minSum = sum;
            topLeftIdx = i;
        }
        if (sum > maxSum) {
            maxSum = sum;
            bottomRightIdx = i;
        }
        if (diff > maxDiff) {
            maxDiff = diff;
            topRightIdx = i;
        }
        if (diff < minDiff) {
            minDiff = diff;
            bottomLeftIdx = i;
        }
    }
    
    // Step 4: Use marker centers for perspective transformation
    vector<Point2f> sourcePoints = {
        markerCenters[topLeftIdx],
        markerCenters[topRightIdx],
        markerCenters[bottomRightIdx],
        markerCenters[bottomLeftIdx]
    };
    
    // Calculate dimensions based on distances between marker centers
    float width1 = norm(sourcePoints[1] - sourcePoints[0]);
    float width2 = norm(sourcePoints[2] - sourcePoints[3]);
    float height1 = norm(sourcePoints[3] - sourcePoints[0]);
    float height2 = norm(sourcePoints[2] - sourcePoints[1]);
    
    float avgWidth = (width1 + width2) / 2.0f;
    float avgHeight = (height1 + height2) / 2.0f;
    
    // Define destination points for the rectified image
    vector<Point2f> destinationPoints = {
        Point2f(0, 0),
        Point2f(avgWidth, 0),
        Point2f(avgWidth, avgHeight),
        Point2f(0, avgHeight)
    };
    
    // Compute homography and warp perspective
    Mat homography = findHomography(sourcePoints, destinationPoints);
    Mat rectifiedImage;
    warpPerspective(inputImage, rectifiedImage, homography, Size(avgWidth, avgHeight));
    
    imshow("Rectified Image", resizeForDisplay(rectifiedImage, DISPLAY_SCALE));
    
    Mat croppedImage = rectifiedImage.clone();
    
    // Step 6: Apply grayscale conversion and adaptive thresholding
    Mat grayImage, binaryImage;
    cvtColor(croppedImage, grayImage, COLOR_BGR2GRAY);
    
    // Apply adaptive thresholding
    adaptiveThreshold(grayImage, binaryImage, 255,
        ADAPTIVE_THRESH_GAUSSIAN_C, 
        THRESH_BINARY_INV,
        111,
        7);
    
    imshow("Thresholded Image", resizeForDisplay(binaryImage, DISPLAY_SCALE));
    
    // Find contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binaryImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    // Create copy for drawing
    Mat result = rectifiedImage.clone();
    
    // Random number generator for colors
    RNG rng(12345);
    
    // Process each contour
    for (size_t i = 0; i < contours.size(); i++) {
        // Generate random color
        // Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        Scalar color = Scalar(0, 0, 0);
        // Approximate contour with polygon
        vector<Point> approx;
        double epsilon = 0.02 * arcLength(contours[i], true);
        approxPolyDP(contours[i], approx, epsilon, true);
        
        // Draw contour and get number of vertices
        drawContours(result, contours, static_cast<int>(i), color, 2);
        int vertices = static_cast<int>(approx.size());
        
        // Calculate centroid for text placement
        Moments m = moments(contours[i]);
        int cx = static_cast<int>(m.m10 / m.m00);
        int cy = static_cast<int>(m.m01 / m.m00);
    }

    imshow("Result with edge", resizeForDisplay(result, DISPLAY_SCALE));

    // Wait for a key press
    waitKey(0);
    return 0;
}
