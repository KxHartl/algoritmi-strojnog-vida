#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

int main() {
    // Step 1: Create image matrix with rectangles
    Mat image = cv::Mat::zeros(9, 9, CV_8UC1);
    rectangle(image, Rect(1, 1, 4, 3), Scalar(255), -1);
    rectangle(image, Rect(7, 1, 1, 1), Scalar(255), -1);
    rectangle(image, Rect(2, 5, 1, 1), Scalar(255), -1);
    rectangle(image, Rect(1, 6, 1, 2), Scalar(255), -1);
    rectangle(image, Rect(2, 7, 1, 1), Scalar(255), -1);
    rectangle(image, Rect(3, 6, 1, 2), Scalar(255), -1);
    rectangle(image, Rect(5, 5, 2, 1), Scalar(255), -1);
    rectangle(image, Rect(6, 6, 1, 1), Scalar(255), -1);
    
    // Step 3: Create kernels for each corner type
    // Top-left corner kernel
    Mat kernel_tl = (Mat_<int>(3,3) << 
        -1, -1, -1,
        -1, 1, 1,
        -1, 1, 1);
        
    // Top-right corner kernel
    Mat kernel_tr = (Mat_<int>(3,3) << 
        -1, -1, -1,
        1, 1, -1,
        1, 1, -1);
        
    // Bottom-left corner kernel
    Mat kernel_bl = (Mat_<int>(3,3) << 
        -1, 1, 1,
        -1, 1, 1,
        -1, -1, -1);
        
    // Bottom-right corner kernel
    Mat kernel_br = (Mat_<int>(3,3) << 
        1, 1, -1,
        1, 1, -1,
        -1, -1, -1);
    
    // Step 4: Apply morphological hit-or-miss for each corner
    Mat hit_tl, hit_tr, hit_bl, hit_br;
    morphologyEx(image, hit_tl, MORPH_HITMISS, kernel_tl);
    morphologyEx(image, hit_tr, MORPH_HITMISS, kernel_tr);
    morphologyEx(image, hit_bl, MORPH_HITMISS, kernel_bl);
    morphologyEx(image, hit_br, MORPH_HITMISS, kernel_br);
    
    // Step 5: Combine results using bitwise_or
    Mat result = Mat::zeros(image.size(), CV_8UC1);
    bitwise_or(hit_tl, hit_tr, result);
    bitwise_or(result, hit_bl, result);
    bitwise_or(result, hit_br, result);
    
    // Step 2: Resize image
    resize(image, image, Size(), 8, 8, INTER_NEAREST);
    resize(result, result, Size(), 8, 8, INTER_NEAREST);


    // Step 6: Display the combined result
    imshow("Original", image);
    imshow("Corner Detection", result);
    
    // Step 7: Write resulting image to a local file
    imwrite("corner_detection.jpg", result);
    
    // Loop until a key is pressed
    while (true) {
        int key = waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
    }
    
    destroyAllWindows();
    return 0;
}
