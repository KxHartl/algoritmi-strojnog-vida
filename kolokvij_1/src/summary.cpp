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
    for (size_t i = 0; i < contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0,256), rng.uniform(0,256), rng.uniform(0,256));
        drawContours(result, contours, static_cast<int>(i), color, 2);
        Rect bounding = boundingRect(contours[i]);
        rectangle(result, bounding, color, 2);
    }
    



    // --- wait for user to exit ---
    while (true) {
        int key = waitKey(1);                   // saving value of presed key to variable "key"
        if (key == 'q' || key == 27) break;     // exiting upon presing "q" or "esc"
    }

    return 0;   // returning "0" represents good exectuing of program
}