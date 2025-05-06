#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // --- Load input image ---
    Mat image = imread("../data/shapes.png", 1);
    if (image.empty()) {
        cout << "Could not open or find the image!" << endl;
        return -1;
    }
    
    // --- Convert to grayscale ---
    Mat image_gray;
    cvtColor(image, image_gray, COLOR_BGR2GRAY);
    
    // --- Adaptive thresholding to obtain binary image ---
    Mat image_binary;
    adaptiveThreshold(image_gray, image_binary, 255,
        ADAPTIVE_THRESH_GAUSSIAN_C,
        THRESH_BINARY_INV,
        105,
        19);
    
    // --- Find external contours in the binary image ---
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(image_binary, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    Mat result = image.clone();

    // --- Draw contours and annotate with number of vertices ---
    RNG rng(12345); // Random color generator
    for (size_t i = 0; i < contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        
        // Approximate contour to polygon and count vertices
        vector<Point> approx;
        double epsilon = 0.02 * arcLength(contours[i], true);
        approxPolyDP(contours[i], approx, epsilon, true);
        
        drawContours(result, contours, static_cast<int>(i), color, 2);

        int vertices = static_cast<int>(approx.size());
        
        // Compute centroid for text placement
        Moments m = moments(contours[i]);
        int cx = static_cast<int>(m.m10 / m.m00);
        int cy = static_cast<int>(m.m01 / m.m00);
        
        // Annotate number of vertices at centroid
        string text = to_string(vertices);
        putText(result, text, Point(cx, cy), FONT_HERSHEY_SIMPLEX, 1, color, 2);
    }
    
    // --- Show results ---
    imshow("Original", image);
    imshow("Grayscale", image_gray);
    imshow("Binary", image_binary);
    imshow("Result with edge count", result);
    
    // --- Wait for user to exit ---
    while (true) {
        int key = waitKey(1);
        if (key == 'q' || key == 27) { // Exit on 'q' or ESC key
            break;
        }   
    }
    
    destroyAllWindows();
    return 0;
}
