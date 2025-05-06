#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

int main() {
    // --- Create a 9x9 binary image and draw various rectangles to form shapes ---
    Mat image = Mat::zeros(9, 9, CV_8UC1);
    rectangle(image, Rect(1, 1, 4, 3), Scalar(255), -1);
    rectangle(image, Rect(7, 1, 1, 1), Scalar(255), -1);
    rectangle(image, Rect(2, 5, 1, 1), Scalar(255), -1);
    rectangle(image, Rect(1, 6, 1, 2), Scalar(255), -1);
    rectangle(image, Rect(2, 7, 1, 1), Scalar(255), -1);
    rectangle(image, Rect(3, 6, 1, 2), Scalar(255), -1);
    rectangle(image, Rect(5, 5, 2, 1), Scalar(255), -1);
    rectangle(image, Rect(6, 6, 1, 1), Scalar(255), -1);

    // --- Define hit-or-miss kernels for each corner type ---
    Mat kernel_tl = (Mat_<int>(3,3) << 
        -1, -1, -1,
        -1, 1, 1,
        -1, 1, 1);
    Mat kernel_tr = (Mat_<int>(3,3) << 
        -1, -1, -1,
         1, 1, -1,
         1, 1, -1);
    Mat kernel_bl = (Mat_<int>(3,3) << 
        -1, 1, 1,
        -1, 1, 1,
        -1, -1, -1);
    Mat kernel_br = (Mat_<int>(3,3) << 
         1, 1, -1,
         1, 1, -1,
        -1, -1, -1);

    // --- Perform hit-or-miss transform for each corner type ---
    Mat hit_tl, hit_tr, hit_bl, hit_br;
    morphologyEx(image, hit_tl, MORPH_HITMISS, kernel_tl);
    morphologyEx(image, hit_tr, MORPH_HITMISS, kernel_tr);
    morphologyEx(image, hit_bl, MORPH_HITMISS, kernel_bl);
    morphologyEx(image, hit_br, MORPH_HITMISS, kernel_br);

    // --- Combine all detected corners into a single result image ---
    Mat result = Mat::zeros(image.size(), CV_8UC1);
    bitwise_or(hit_tl, hit_tr, result);
    bitwise_or(result, hit_bl, result);
    bitwise_or(result, hit_br, result);

    // --- Enlarge images for visualization ---
    resize(image, image, Size(), 64, 64, INTER_NEAREST);
    resize(result, result, Size(), 64, 64, INTER_NEAREST);

    // --- Display and save results ---
    imshow("Original", image);
    imshow("Corner Detection", result);
    imwrite("../results/corner_detection.jpg", result);

    // --- Wait for user to exit ---
    while (true) {
        int key = waitKey(1);
        if (key == 'q' || key == 27) break;
    }
    destroyAllWindows();
    return 0;
}
