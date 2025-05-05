#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // Load color image
    Mat image = imread("../data/shapes.png", 1);
    if (image.empty()) {
        cout << "Could not open or find the image!" << endl;
        return -1;
    }
    
    // Convert to grayscale
    Mat image_gray;
    cvtColor(image, image_gray, COLOR_BGR2GRAY);
    
    // Apply thresholding to create binary image
    Mat image_binary;
    // threshold(image_gray, image_binary, 128, 255, THRESH_BINARY_INV);
    adaptiveThreshold(
        image_gray,
        image_binary,
        255,
        ADAPTIVE_THRESH_GAUSSIAN_C,
        THRESH_BINARY_INV,
        105,
        19);
    
    // Find contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(image_binary, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    // Create copy for drawing
    Mat result = image.clone();
    
    // Random number generator for colors
    RNG rng(12345);
    
    // Process each contour
    for (size_t i = 0; i < contours.size(); i++) {
        // Generate random color
        Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        
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
        
        // Write the number of edges next to the shape
        string text = to_string(vertices);
        putText(result, text, Point(cx, cy), FONT_HERSHEY_SIMPLEX, 1, color, 2);
    }
    
    // Display results
    imshow("Original", image);
    imshow("Grayscale", image_gray);
    imshow("Binary", image_binary);
    imshow("Result with edge count", result);
    
    waitKey(0);
    return 0;
}
