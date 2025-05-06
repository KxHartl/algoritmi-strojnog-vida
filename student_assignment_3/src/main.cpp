#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

// --- Kernel and filter parameters for each scenario ---
int kernel_mean_original = 3, kernel_gauss_original = 3, kernel_median_original = 3;
int kernel_mean_gaussian_noise = 3, kernel_gauss_gaussian_noise = 3, kernel_median_gaussian_noise = 3;
int kernel_mean_saltpepper = 3, kernel_gauss_saltpepper = 3, kernel_median_saltpepper = 3;

int d_bilateral_original = 5, sigmaColor_bilateral_original = 50, sigmaSpace_bilateral_original = 50;
int d_bilateral_gaussian_noise = 5, sigmaColor_bilateral_gaussian_noise = 50, sigmaSpace_bilateral_gaussian_noise = 50;
int d_bilateral_saltpepper = 5, sigmaColor_bilateral_saltpepper = 50, sigmaSpace_bilateral_saltpepper = 50;

// --- Output images for each filter and noise scenario ---
Mat filtered_mean_original, filtered_gauss_original, filtered_median_original, filtered_bilateral_original;
Mat filtered_mean_gaussian_noise, filtered_gauss_gaussian_noise, filtered_median_gaussian_noise, filtered_bilateral_gaussian_noise;
Mat filtered_mean_saltpepper, filtered_gauss_saltpepper, filtered_median_saltpepper, filtered_bilateral_saltpepper;

// --- Ensure kernel size is always odd (required by most filters) ---
int ensureOdd(int value) {
    return (value % 2 == 0) ? value + 1 : value;
}

// --- Filter callbacks: update filter output and window on trackbar change ---
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
void onBilateralOriginalDTrackbar(int, void* userdata) {
    d_bilateral_original = ensureOdd(d_bilateral_original);
    bilateralFilter(*(Mat*)userdata, filtered_bilateral_original, d_bilateral_original, sigmaColor_bilateral_original, sigmaSpace_bilateral_original);
    imshow("Bilateral Filter - Original", filtered_bilateral_original);
}
void onBilateralOriginalSigmaColorTrackbar(int, void* userdata) {
    bilateralFilter(*(Mat*)userdata, filtered_bilateral_original, d_bilateral_original, sigmaColor_bilateral_original, sigmaSpace_bilateral_original);
    imshow("Bilateral Filter - Original", filtered_bilateral_original);
}
void onBilateralOriginalSigmaSpaceTrackbar(int, void* userdata) {
    bilateralFilter(*(Mat*)userdata, filtered_bilateral_original, d_bilateral_original, sigmaColor_bilateral_original, sigmaSpace_bilateral_original);
    imshow("Bilateral Filter - Original", filtered_bilateral_original);
}

// --- Repeat similar filter callbacks for Gaussian noise and Salt & Pepper noise images ---
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
void onBilateralGaussianNoiseDTrackbar(int, void* userdata) {
    d_bilateral_gaussian_noise = ensureOdd(d_bilateral_gaussian_noise);
    bilateralFilter(*(Mat*)userdata, filtered_bilateral_gaussian_noise, d_bilateral_gaussian_noise, sigmaColor_bilateral_gaussian_noise, sigmaSpace_bilateral_gaussian_noise);
    imshow("Bilateral Filter - Gaussian Noise", filtered_bilateral_gaussian_noise);
}
void onBilateralGaussianNoiseSigmaColorTrackbar(int, void* userdata) {
    bilateralFilter(*(Mat*)userdata, filtered_bilateral_gaussian_noise, d_bilateral_gaussian_noise, sigmaColor_bilateral_gaussian_noise, sigmaSpace_bilateral_gaussian_noise);
    imshow("Bilateral Filter - Gaussian Noise", filtered_bilateral_gaussian_noise);
}
void onBilateralGaussianNoiseSigmaSpaceTrackbar(int, void* userdata) {
    bilateralFilter(*(Mat*)userdata, filtered_bilateral_gaussian_noise, d_bilateral_gaussian_noise, sigmaColor_bilateral_gaussian_noise, sigmaSpace_bilateral_gaussian_noise);
    imshow("Bilateral Filter - Gaussian Noise", filtered_bilateral_gaussian_noise);
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
void onBilateralSaltPepperDTrackbar(int, void* userdata) {
    d_bilateral_saltpepper = ensureOdd(d_bilateral_saltpepper);
    bilateralFilter(*(Mat*)userdata, filtered_bilateral_saltpepper, d_bilateral_saltpepper, sigmaColor_bilateral_saltpepper, sigmaSpace_bilateral_saltpepper);
    imshow("Bilateral Filter - Salt & Pepper", filtered_bilateral_saltpepper);
}
void onBilateralSaltPepperSigmaColorTrackbar(int, void* userdata) {
    bilateralFilter(*(Mat*)userdata, filtered_bilateral_saltpepper, d_bilateral_saltpepper, sigmaColor_bilateral_saltpepper, sigmaSpace_bilateral_saltpepper);
    imshow("Bilateral Filter - Salt & Pepper", filtered_bilateral_saltpepper);
}
void onBilateralSaltPepperSigmaSpaceTrackbar(int, void* userdata) {
    bilateralFilter(*(Mat*)userdata, filtered_bilateral_saltpepper, d_bilateral_saltpepper, sigmaColor_bilateral_saltpepper, sigmaSpace_bilateral_saltpepper);
    imshow("Bilateral Filter - Salt & Pepper", filtered_bilateral_saltpepper);
}

int main() {
    // --- Load and resize grayscale image ---
    Mat image = imread("../data/parts.png", 0);
    if (image.empty()) {
        cout << "Error: Could not load the image!" << endl;
        return -1;
    }
    resize(image, image, Size(), 0.5, 0.5, INTER_LINEAR);

    // --- Create noisy versions of the image ---
    Mat image_gaussian_noise = image.clone();
    Mat image_salt_and_pepper_noise = image.clone();

    // Add Gaussian noise
    Mat gaussian_noise = Mat(image.size(), image.type());
    randn(gaussian_noise, 50, 10);
    add(image, gaussian_noise, image_gaussian_noise);

    // Add Salt & Pepper noise
    Mat saltpepper_noise = Mat::zeros(image.rows, image.cols, CV_8U);
    randu(saltpepper_noise, 0, 255);
    Mat black = saltpepper_noise < 10;
    Mat white = saltpepper_noise > 245;
    image_salt_and_pepper_noise.setTo(255, white);
    image_salt_and_pepper_noise.setTo(0, black);

    // --- Initial filtering for all images and filters ---
    blur(image, filtered_mean_original, Size(kernel_mean_original, kernel_mean_original));
    GaussianBlur(image, filtered_gauss_original, Size(kernel_gauss_original, kernel_gauss_original), 0);
    medianBlur(image, filtered_median_original, kernel_median_original);
    bilateralFilter(image, filtered_bilateral_original, d_bilateral_original, sigmaColor_bilateral_original, sigmaSpace_bilateral_original);

    blur(image_gaussian_noise, filtered_mean_gaussian_noise, Size(kernel_mean_gaussian_noise, kernel_mean_gaussian_noise));
    GaussianBlur(image_gaussian_noise, filtered_gauss_gaussian_noise, Size(kernel_gauss_gaussian_noise, kernel_gauss_gaussian_noise), 0);
    medianBlur(image_gaussian_noise, filtered_median_gaussian_noise, kernel_median_gaussian_noise);
    bilateralFilter(image_gaussian_noise, filtered_bilateral_gaussian_noise, d_bilateral_gaussian_noise, sigmaColor_bilateral_gaussian_noise, sigmaSpace_bilateral_gaussian_noise);

    blur(image_salt_and_pepper_noise, filtered_mean_saltpepper, Size(kernel_mean_saltpepper, kernel_mean_saltpepper));
    GaussianBlur(image_salt_and_pepper_noise, filtered_gauss_saltpepper, Size(kernel_gauss_saltpepper, kernel_gauss_saltpepper), 0);
    medianBlur(image_salt_and_pepper_noise, filtered_median_saltpepper, kernel_median_saltpepper);
    bilateralFilter(image_salt_and_pepper_noise, filtered_bilateral_saltpepper, d_bilateral_saltpepper, sigmaColor_bilateral_saltpepper, sigmaSpace_bilateral_saltpepper);

    // --- Show original and noisy images ---
    imshow("Original", image);
    imshow("Gaussian Noise", image_gaussian_noise);
    imshow("Salt and Pepper", image_salt_and_pepper_noise);

    // --- Create windows and trackbars for each filter and scenario ---
    // Original
    namedWindow("Mean Filter - Original", WINDOW_AUTOSIZE);
    namedWindow("Gaussian Filter - Original", WINDOW_AUTOSIZE);
    namedWindow("Median Filter - Original", WINDOW_AUTOSIZE);
    namedWindow("Bilateral Filter - Original", WINDOW_AUTOSIZE);
    createTrackbar("Kernel Size", "Mean Filter - Original", &kernel_mean_original, 15, onMeanOriginalTrackbar, &image);
    createTrackbar("Kernel Size", "Gaussian Filter - Original", &kernel_gauss_original, 15, onGaussOriginalTrackbar, &image);
    createTrackbar("Kernel Size", "Median Filter - Original", &kernel_median_original, 15, onMedianOriginalTrackbar, &image);
    createTrackbar("Diameter", "Bilateral Filter - Original", &d_bilateral_original, 15, onBilateralOriginalDTrackbar, &image);
    createTrackbar("Sigma Color", "Bilateral Filter - Original", &sigmaColor_bilateral_original, 150, onBilateralOriginalSigmaColorTrackbar, &image);
    createTrackbar("Sigma Space", "Bilateral Filter - Original", &sigmaSpace_bilateral_original, 150, onBilateralOriginalSigmaSpaceTrackbar, &image);

    // Gaussian Noise
    namedWindow("Mean Filter - Gaussian Noise", WINDOW_AUTOSIZE);
    namedWindow("Gaussian Filter - Gaussian Noise", WINDOW_AUTOSIZE);
    namedWindow("Median Filter - Gaussian Noise", WINDOW_AUTOSIZE);
    namedWindow("Bilateral Filter - Gaussian Noise", WINDOW_AUTOSIZE);
    createTrackbar("Kernel Size", "Mean Filter - Gaussian Noise", &kernel_mean_gaussian_noise, 15, onMeanGaussianNoiseTrackbar, &image_gaussian_noise);
    createTrackbar("Kernel Size", "Gaussian Filter - Gaussian Noise", &kernel_gauss_gaussian_noise, 15, onGaussGaussianNoiseTrackbar, &image_gaussian_noise);
    createTrackbar("Kernel Size", "Median Filter - Gaussian Noise", &kernel_median_gaussian_noise, 15, onMedianGaussianNoiseTrackbar, &image_gaussian_noise);
    createTrackbar("Diameter", "Bilateral Filter - Gaussian Noise", &d_bilateral_gaussian_noise, 15, onBilateralGaussianNoiseDTrackbar, &image_gaussian_noise);
    createTrackbar("Sigma Color", "Bilateral Filter - Gaussian Noise", &sigmaColor_bilateral_gaussian_noise, 150, onBilateralGaussianNoiseSigmaColorTrackbar, &image_gaussian_noise);
    createTrackbar("Sigma Space", "Bilateral Filter - Gaussian Noise", &sigmaSpace_bilateral_gaussian_noise, 150, onBilateralGaussianNoiseSigmaSpaceTrackbar, &image_gaussian_noise);

    // Salt & Pepper Noise
    namedWindow("Mean Filter - Salt & Pepper", WINDOW_AUTOSIZE);
    namedWindow("Gaussian Filter - Salt & Pepper", WINDOW_AUTOSIZE);
    namedWindow("Median Filter - Salt & Pepper", WINDOW_AUTOSIZE);
    namedWindow("Bilateral Filter - Salt & Pepper", WINDOW_AUTOSIZE);
    createTrackbar("Kernel Size", "Mean Filter - Salt & Pepper", &kernel_mean_saltpepper, 15, onMeanSaltPepperTrackbar, &image_salt_and_pepper_noise);
    createTrackbar("Kernel Size", "Gaussian Filter - Salt & Pepper", &kernel_gauss_saltpepper, 15, onGaussSaltPepperTrackbar, &image_salt_and_pepper_noise);
    createTrackbar("Kernel Size", "Median Filter - Salt & Pepper", &kernel_median_saltpepper, 15, onMedianSaltPepperTrackbar, &image_salt_and_pepper_noise);
    createTrackbar("Diameter", "Bilateral Filter - Salt & Pepper", &d_bilateral_saltpepper, 15, onBilateralSaltPepperDTrackbar, &image_salt_and_pepper_noise);
    createTrackbar("Sigma Color", "Bilateral Filter - Salt & Pepper", &sigmaColor_bilateral_saltpepper, 150, onBilateralSaltPepperSigmaColorTrackbar, &image_salt_and_pepper_noise);
    createTrackbar("Sigma Space", "Bilateral Filter - Salt & Pepper", &sigmaSpace_bilateral_saltpepper, 150, onBilateralSaltPepperSigmaSpaceTrackbar, &image_salt_and_pepper_noise);

    // --- Show all filtered images initially ---
    imshow("Mean Filter - Original", filtered_mean_original);
    imshow("Gaussian Filter - Original", filtered_gauss_original);
    imshow("Median Filter - Original", filtered_median_original);
    imshow("Bilateral Filter - Original", filtered_bilateral_original);
    imshow("Mean Filter - Gaussian Noise", filtered_mean_gaussian_noise);
    imshow("Gaussian Filter - Gaussian Noise", filtered_gauss_gaussian_noise);
    imshow("Median Filter - Gaussian Noise", filtered_median_gaussian_noise);
    imshow("Bilateral Filter - Gaussian Noise", filtered_bilateral_gaussian_noise);
    imshow("Mean Filter - Salt & Pepper", filtered_mean_saltpepper);
    imshow("Gaussian Filter - Salt & Pepper", filtered_gauss_saltpepper);
    imshow("Median Filter - Salt & Pepper", filtered_median_saltpepper);
    imshow("Bilateral Filter - Salt & Pepper", filtered_bilateral_saltpepper);

    // --- Main loop: wait for user to exit ---
    while (true) {
        int key = waitKey(1);
        if (key == 'q' || key == 27) break;
    }

    destroyAllWindows();
    return 0;
}
