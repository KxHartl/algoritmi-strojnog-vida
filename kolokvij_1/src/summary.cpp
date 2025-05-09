#include <opencv2/opencv.hpp>   // core OpenCV functionality
#include <opencv2/aruco.hpp>    // ArUco marker
#include <iostream>             // input - output
#include <vector>               // vectors, dynamic arrays...
#include <cmath>                // extended mathematical operations...
#include <algorithm>            // sorting, searching, counting...
#include <limits>               // min, max...

using namespace cv;             // namespace for opencv
using namespace std;            // namespace for standard c++ headers


// --- Global variables --- //
const int globalInteger = 10;
const char globalChar = 'H';
vector<int> globalVector = {1, 2, 3};


// --- Functions --- //
int add_and_insert(int addNumber){
    globalVector.push_back(globalInteger + addNumber);

    return 0;
}

void on_trackbar(int, void*) {
     
}



// --- Main function --- //
int main(void){
    //######################################################################################################################
    // // --- For loop --- //
    // int size = globalVector.size();
    // for (int i = 0; i < 10; i++){
    //     add_and_insert(i);
    //     cout << i << "\t" << globalVector.at(size+ i) << "\n";
    // }

    // // --- While loop --- //
    // string userInput;
    // while (true){
    //     cout << "Input anything:\n";
    //     getline(cin, userInput);
    //     cout << "User inputted:\n" << userInput << "\n\n";

    //     if (userInput == "exit" || userInput == "quit"){
    //         break;
    //     }
    // }



    // --- OpenCV --- //#########################################################################################################################


    // --- Input --- //
    Mat inputImage = imread("../data/image_01.png", IMREAD_COLOR);
    if (inputImage.empty()){
        cout << "Could not open or find the image!";
        return -1;
    }

    Mat grayImage, hsvImage;
    cvtColor(inputImage, grayImage, COLOR_BGR2GRAY);
    cvtColor(inputImage, hsvImage, COLOR_BGR2HSV);

    Mat smallImage, largeImage;
    resize(inputImage, smallImage, Size(), 0.5, 0.5, INTER_LINEAR);
    resize(inputImage, largeImage, Size(800, 400), 0, 0, INTER_LINEAR);


    namedWindow("Small Image", WINDOW_AUTOSIZE);
    namedWindow("Large Image", WINDOW_AUTOSIZE);

    imshow("Small Image", smallImage);
    imshow("Large Image", largeImage);

    // --- Adding noise --- // ######################################################################################################################
    Mat noisyImage = inputImage.clone();
    
    // --- Gaussian noise --- //
    Mat noise = Mat(inputImage.size(), inputImage.type());
    randn(noise, 50, 10); // mean=50 Controls the brightness shift in the noise (typically 0-255). Higher values make the image brighter
                            //  stddev=10 Controls the intensity of noise. Higher values create more pronounced noise
    add(inputImage, noise, noisyImage);

    // -- Salt and Pepper noise --- //
    Mat saltpepperNoise = Mat::zeros(inputImage.rows, inputImage.cols, CV_8U);
    randu(saltpepperNoise, 0, 255);
    Mat black = saltpepperNoise < 10, white = saltpepperNoise > 245;
    noisyImage.setTo(255, white);
    noisyImage.setTo(0, black);

    // --- Filters --- //##########################################################################################################################
    Mat filteredImage;
    int kernel_size = 3;

    // --- Mean (Averaging) Filter --- //
    blur(noisyImage, filteredImage, Size(kernel_size, kernel_size));

    // --- Gaussian Filter --- //
    GaussianBlur(noisyImage, filteredImage, Size(kernel_size, kernel_size), 0);

    // --- Median Filter --- //
    medianBlur(noisyImage, filteredImage, kernel_size);

    // --- Bilateral Filter --- //
    int d = 5, sigmaColor = 50, sigmaSpace = 50;
    bilateralFilter(noisyImage, filteredImage, d, sigmaColor, sigmaSpace);

    // --- HSV Filtering --- //
    int h_min = 10, s_min = 10, v_min = 10, h_max = 200, s_max = 200, v_max = 200;
    Mat hsvMask;
    inRange(hsvImage, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), hsvMask);


    // --- Thresholding and Morphology --- //#######################################################################################################
    Mat binaryImage;
    int blockSize = 3, C = 25;
    adaptiveThreshold(grayImage, binaryImage, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, blockSize, C);

    Mat morphImage;
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(kernel_size, kernel_size));
    morphologyEx(binaryImage, morphImage, MORPH_OPEN, kernel);


    // --- Contour Detection and Analysis --- //####################################################################################################
    // --- Find Contours --- //
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binaryImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // --- Draw Contours and Bounding Boxes --- //
    Mat resultsImage;
    RNG rng(12345);
    for (size_t i = 0; i < contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0,256), rng.uniform(0,256), rng.uniform(0,256));
        drawContours(resultsImage, contours, static_cast<int>(i), color, 2);
        Rect bounding = boundingRect(contours[i]);
        rectangle(resultsImage, bounding, color, 2);
    }

    // --- Edge and Line Detection --- //#############################################################################################################
    // --- Canny lines --- //
    Mat edges;
    int threshold1 = 150, threshold2 = 500, apertureSize = 3;
    Canny(inputImage, edges, threshold1, threshold2, apertureSize);

    // --- Hough Lines --- //
    resultsImage = inputImage.clone();
    vector<Vec4i> lines;
    int houghThreshold = 20, minLineLength = 40, maxLineGap = 1;
    HoughLinesP(edges, lines, 1, CV_PI/180, houghThreshold, minLineLength, maxLineGap);
    for (auto& l : lines) {
        line(resultsImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2);
    }

    // --- ArUco Marker --- //#####################################################################################################################
    // --- Detect Markers --- //
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

    // Visualize detected markers
    aruco::drawDetectedMarkers(visualizationImage, markerCorners, markerIds);

    // --- Find Homography and Warp --- //
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


    // --- Widows and track bar --- //##############################################################################################################
    namedWindow("Window", WINDOW_AUTOSIZE);

    int param = 3, max_value = 11;
    createTrackbar("Param Name", "WindowName", &param, max_value, on_trackbar);

    // --- wait for user to exit ---
    while (true) {
        int key = waitKey(1);                   // saving value of presed key to variable "key"
        if (key == 'q' || key == 27) break;     // exiting upon presing "q" or "esc"
    }

    return 0;   // returning "0" represents good exectuing of program
}