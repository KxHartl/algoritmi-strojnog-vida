#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

// Global variables for kernel sizes (must be odd, starting with 3)
int kernel_mean_original = 3;
int kernel_gauss_original = 3;
int kernel_median_original = 3;
int kernel_mean_gaussian_noise = 3;
int kernel_gauss_gaussian_noise = 3;
int kernel_median_gaussian_noise = 3;
int kernel_mean_saltpepper = 3;
int kernel_gauss_saltpepper = 3;
int kernel_median_saltpepper = 3;

// Global Mats to store filtered images
Mat filtered_mean_original, filtered_gauss_original, filtered_median_original;
Mat filtered_mean_gaussian_noise, filtered_gauss_gaussian_noise, filtered_median_gaussian_noise;
Mat filtered_mean_saltpepper, filtered_gauss_saltpepper, filtered_median_saltpepper;

// Function to ensure kernel size is odd
int ensureOdd(int value) {
    return (value % 2 == 0) ? value + 1 : value;
}

// Callback functions for trackbars (update filtered images on change)
void onMeanOriginalTrackbar(int, void* userdata) {
    kernel_mean_original = ensureOdd(kernel_mean_original);
    blur(*(Mat*)userdata, filtered_mean_original, Size(kernel_mean_original, kernel_mean_original));
    imshow("Mean Filter - Original", filtered_mean_original);
}

void onGaussOriginalTrackbar(int, void* userdata) {
    kernel_gauss_original = ensureOdd(kernel_gauss_original);
    GaussianBlur(*(Mat*)userdata, filtered_gauss_original, Size(kernel_gauss_original, kernel_gauss_original), 0);
    imshow("Gaussian Filter - Original", filtered_gauss_original);
}

void onMedianOriginalTrackbar(int, void* userdata) {
    kernel_median_original = ensureOdd(kernel_median_original);
    medianBlur(*(Mat*)userdata, filtered_median_original, kernel_median_original);
    imshow("Median Filter - Original", filtered_median_original);
}

void onMeanGaussianNoiseTrackbar(int, void* userdata) {
    kernel_mean_gaussian_noise = ensureOdd(kernel_mean_gaussian_noise);
    blur(*(Mat*)userdata, filtered_mean_gaussian_noise, Size(kernel_mean_gaussian_noise, kernel_mean_gaussian_noise));
    imshow("Mean Filter - Gaussian Noise", filtered_mean_gaussian_noise);
}

void onGaussGaussianNoiseTrackbar(int, void* userdata) {
    kernel_gauss_gaussian_noise = ensureOdd(kernel_gauss_gaussian_noise);
    GaussianBlur(*(Mat*)userdata, filtered_gauss_gaussian_noise, Size(kernel_gauss_gaussian_noise, kernel_gauss_gaussian_noise), 0);
    imshow("Gaussian Filter - Gaussian Noise", filtered_gauss_gaussian_noise);
}

void onMedianGaussianNoiseTrackbar(int, void* userdata) {
    kernel_median_gaussian_noise = ensureOdd(kernel_median_gaussian_noise);
    medianBlur(*(Mat*)userdata, filtered_median_gaussian_noise, kernel_median_gaussian_noise);
    imshow("Median Filter - Gaussian Noise", filtered_median_gaussian_noise);
}

void onMeanSaltPepperTrackbar(int, void* userdata) {
    kernel_mean_saltpepper = ensureOdd(kernel_mean_saltpepper);
    blur(*(Mat*)userdata, filtered_mean_saltpepper, Size(kernel_mean_saltpepper, kernel_mean_saltpepper));
    imshow("Mean Filter - Salt & Pepper", filtered_mean_saltpepper);
}

void onGaussSaltPepperTrackbar(int, void* userdata) {
    kernel_gauss_saltpepper = ensureOdd(kernel_gauss_saltpepper);
    GaussianBlur(*(Mat*)userdata, filtered_gauss_saltpepper, Size(kernel_gauss_saltpepper, kernel_gauss_saltpepper), 0);
    imshow("Gaussian Filter - Salt & Pepper", filtered_gauss_saltpepper);
}

void onMedianSaltPepperTrackbar(int, void* userdata) {
    kernel_median_saltpepper = ensureOdd(kernel_median_saltpepper);
    medianBlur(*(Mat*)userdata, filtered_median_saltpepper, kernel_median_saltpepper);
    imshow("Median Filter - Salt & Pepper", filtered_median_saltpepper);
}

int main() {
    Mat image = imread("../data/parts.png", 0);
    if (image.empty()) {
        cout << "Error: Could not load the image!" << endl;
        return -1;
    }
    resize(image, image, Size(), 0.25, 0.25, INTER_LINEAR);

    Mat image_gaussian_noise = image.clone();
    Mat image_salt_and_pepper_noise = image.clone();

    // Gaussian noise
    Mat gaussian_noise = Mat(image.size(), image.type());
    randn(gaussian_noise, 50, 10);
    add(image, gaussian_noise, image_gaussian_noise);
    
    // Salt and pepper noise
    Mat saltpepper_noise = Mat::zeros(image.rows, image.cols, CV_8U);
    randu(saltpepper_noise, 0, 255);
    Mat black = saltpepper_noise < 10;
    Mat white = saltpepper_noise > 245;
    image_salt_and_pepper_noise.setTo(255, white);
    image_salt_and_pepper_noise.setTo(0, black);

    // Display original and noisy images
    imshow("Original", image);
    imshow("Gaussian Noise", image_gaussian_noise);
    imshow("Salt and Pepper", image_salt_and_pepper_noise);

    // Initialize filtered images for original image
    blur(image, filtered_mean_original, Size(kernel_mean_original, kernel_mean_original));
    GaussianBlur(image, filtered_gauss_original, Size(kernel_gauss_original, kernel_gauss_original), 0);
    medianBlur(image, filtered_median_original, kernel_median_original);

    // Initialize filtered images for Gaussian noise image
    blur(image_gaussian_noise, filtered_mean_gaussian_noise, Size(kernel_mean_gaussian_noise, kernel_mean_gaussian_noise));
    GaussianBlur(image_gaussian_noise, filtered_gauss_gaussian_noise, Size(kernel_gauss_gaussian_noise, kernel_gauss_gaussian_noise), 0);
    medianBlur(image_gaussian_noise, filtered_median_gaussian_noise, kernel_median_gaussian_noise);

    // Initialize filtered images for Salt & Pepper noise image
    blur(image_salt_and_pepper_noise, filtered_mean_saltpepper, Size(kernel_mean_saltpepper, kernel_mean_saltpepper));
    GaussianBlur(image_salt_and_pepper_noise, filtered_gauss_saltpepper, Size(kernel_gauss_saltpepper, kernel_gauss_saltpepper), 0);
    medianBlur(image_salt_and_pepper_noise, filtered_median_saltpepper, kernel_median_saltpepper);

    // Create windows for filtered images
    namedWindow("Mean Filter - Original", WINDOW_AUTOSIZE);
    namedWindow("Gaussian Filter - Original", WINDOW_AUTOSIZE);
    namedWindow("Median Filter - Original", WINDOW_AUTOSIZE);
    namedWindow("Mean Filter - Gaussian Noise", WINDOW_AUTOSIZE);
    namedWindow("Gaussian Filter - Gaussian Noise", WINDOW_AUTOSIZE);
    namedWindow("Median Filter - Gaussian Noise", WINDOW_AUTOSIZE);
    namedWindow("Mean Filter - Salt & Pepper", WINDOW_AUTOSIZE);
    namedWindow("Gaussian Filter - Salt & Pepper", WINDOW_AUTOSIZE);
    namedWindow("Median Filter - Salt & Pepper", WINDOW_AUTOSIZE);

    // Create trackbars for kernel size adjustment (range 1-15, enforced to be odd)
    createTrackbar("Kernel Size", "Mean Filter - Original", &kernel_mean_original, 15, onMeanOriginalTrackbar, &image);
    createTrackbar("Kernel Size", "Gaussian Filter - Original", &kernel_gauss_original, 15, onGaussOriginalTrackbar, &image);
    createTrackbar("Kernel Size", "Median Filter - Original", &kernel_median_original, 15, onMedianOriginalTrackbar, &image);
    createTrackbar("Kernel Size", "Mean Filter - Gaussian Noise", &kernel_mean_gaussian_noise, 15, onMeanGaussianNoiseTrackbar, &image_gaussian_noise);
    createTrackbar("Kernel Size", "Gaussian Filter - Gaussian Noise", &kernel_gauss_gaussian_noise, 15, onGaussGaussianNoiseTrackbar, &image_gaussian_noise);
    createTrackbar("Kernel Size", "Median Filter - Gaussian Noise", &kernel_median_gaussian_noise, 15, onMedianGaussianNoiseTrackbar, &image_gaussian_noise);
    createTrackbar("Kernel Size", "Mean Filter - Salt & Pepper", &kernel_mean_saltpepper, 15, onMeanSaltPepperTrackbar, &image_salt_and_pepper_noise);
    createTrackbar("Kernel Size", "Gaussian Filter - Salt & Pepper", &kernel_gauss_saltpepper, 15, onGaussSaltPepperTrackbar, &image_salt_and_pepper_noise);
    createTrackbar("Kernel Size", "Median Filter - Salt & Pepper", &kernel_median_saltpepper, 15, onMedianSaltPepperTrackbar, &image_salt_and_pepper_noise);

    // Display initial filtered images
    imshow("Mean Filter - Original", filtered_mean_original);
    imshow("Gaussian Filter - Original", filtered_gauss_original);
    imshow("Median Filter - Original", filtered_median_original);
    imshow("Mean Filter - Gaussian Noise", filtered_mean_gaussian_noise);
    imshow("Gaussian Filter - Gaussian Noise", filtered_gauss_gaussian_noise);
    imshow("Median Filter - Gaussian Noise", filtered_median_gaussian_noise);
    imshow("Mean Filter - Salt & Pepper", filtered_mean_saltpepper);
    imshow("Gaussian Filter - Salt & Pepper", filtered_gauss_saltpepper);
    imshow("Median Filter - Salt & Pepper", filtered_median_saltpepper);

    // Loop until a key is pressed or windows are closed
    while (true) {
        int key = waitKey(1); // Wait for 1 ms for any key press
        if (key == 'q' || key == 27) { // Exit on 'q' or ESC key
            break;
        }
    }

    destroyAllWindows();
    return 0;
}
    