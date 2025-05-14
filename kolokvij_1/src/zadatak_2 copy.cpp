#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

// --- Controls for each region (fixed values) ---
struct RegionControls {
    int filter_type = 0; // 0: Gaussian, 1: Median, 2: Bilateral, 3: None
    int blur_ksize = 13;
    int bilateral_d = 9;
    int bilateral_sigmaColor = 75;
    int bilateral_sigmaSpace = 75;
    int thresh_blocksize = 41;
    int thresh_C = 11;
    int invert = 0;
};

// --- Global controls ---
RegionControls upper_left, upper_right, lower;
Mat img, upper_left_img, upper_right_img, lower_img;
Mat upper_left_binary, upper_right_binary, lower_binary;
Mat result_img;

// Helper to ensure odd kernel size
inline int odd(int v) { return (v % 2 == 1) ? v : v + 1; }

void process_region(
    const Mat& part, Mat& binary,
    const RegionControls& ctrl,
    Scalar color, Point offset, Mat& result_img
) {
    // 1. Filtering
    Mat filtered = part.clone();
    if (ctrl.filter_type == 0) { // Gaussian
        int ksize = max(3, odd(ctrl.blur_ksize));
        GaussianBlur(part, filtered, Size(ksize, ksize), 0);
    } else if (ctrl.filter_type == 1) { // Median
        int ksize = max(3, odd(ctrl.blur_ksize));
        medianBlur(part, filtered, ksize);
    } else if (ctrl.filter_type == 2) { // Bilateral
        bilateralFilter(part, filtered, ctrl.bilateral_d, ctrl.bilateral_sigmaColor, ctrl.bilateral_sigmaSpace);
    } // else: None

    // 2. Grayscale
    Mat gray;
    cvtColor(filtered, gray, COLOR_BGR2GRAY);

    // 3. Adaptive threshold
    int blockSize = max(3, odd(ctrl.thresh_blocksize));
    adaptiveThreshold(gray, binary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, blockSize, ctrl.thresh_C);

    // 4. Invert if requested
    if (ctrl.invert) {
        bitwise_not(binary, binary);
    }

    // 5. Find contours and draw
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binary, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size(); ++i) {
        vector<Point> approx;
        double peri = arcLength(contours[i], true);
        approxPolyDP(contours[i], approx, 0.02 * peri, true);
        if (approx.size() == 4 && isContourConvex(approx) && contourArea(approx) > 500) {
            Rect rect = boundingRect(approx);
            Point center(rect.x + rect.width / 2, rect.y + rect.height / 2);

            // Draw on result_img with offset
            rectangle(result_img, Rect(rect.x + offset.x, rect.y + offset.y, rect.width, rect.height), color, 2);
            drawContours(result_img, contours, (int)i, color, 2, LINE_8, noArray(), 0, offset);
            circle(result_img, Point(center.x + offset.x, center.y + offset.y), 4, Scalar(0, 0, 255), -1);
            
            // Output center coordinates and edge count
            cout << "Part detected at " << center.x + offset.x << "," << center.y + offset.y 
                 << " with " << approx.size() << " edges" << endl;
        }
    }
}

// Process all regions and update windows
void process_all() {
    result_img = img.clone();

    // Process upper left
    process_region(
        upper_left_img, upper_left_binary, upper_left,
        Scalar(0,255,0), Point(0,0), result_img
    );
    imshow("Upper Left Binary", upper_left_binary);

    // Process upper right
    process_region(
        upper_right_img, upper_right_binary, upper_right,
        Scalar(0,255,0), Point(upper_left_img.cols,0), result_img
    );
    imshow("Upper Right Binary", upper_right_binary);

    // Process lower
    process_region(
        lower_img, lower_binary, lower,
        Scalar(0,255,0), Point(0,upper_left_img.rows), result_img
    );
    imshow("Lower Binary", lower_binary);

    // Show combined result
    imshow("Combined Result", result_img);
}

int main() {
    img = imread("../data/12.jpg");
    if (img.empty()) {
        cout << "Image not found!" << endl;
        return -1;
    }
    resize(img, img, Size(), 0.25, 0.25, INTER_LINEAR);

    // Split image into parts
    int split_row = img.rows / 2;
    int split_col = img.cols / 2;
    int bias_ud = -80;
    int bias_lr = -70;
    upper_left_img  = img(Rect(0, 0, split_col + bias_lr, split_row + bias_ud)).clone();
    upper_right_img = img(Rect(split_col + bias_lr, 0, img.cols - split_col - bias_lr, split_row + bias_ud)).clone();
    lower_img       = img(Rect(0, split_row + bias_ud, img.cols, img.rows - split_row - bias_ud)).clone();

    // Set specific parameters for each region (optional - customize as needed)
    upper_left.filter_type = 0;  // Gaussian
    upper_left.blur_ksize = 9;
    upper_left.thresh_blocksize = 44;
    upper_left.thresh_C = 0;
    upper_left.invert = 1;
    
    upper_right.filter_type = 2;  // Median
    upper_right.bilateral_d = 19;
    upper_right.bilateral_sigmaColor = 102;
    upper_right.thresh_blocksize = 41;
    upper_right.thresh_C = 6;
    upper_right.invert = 1;

    
    lower.filter_type = 3;  // None
    lower.thresh_blocksize = 54;
    lower.thresh_C = 2;
    lower.invert = 0;


    // Prepare result image
    result_img = img.clone();

    // Create windows
    namedWindow("Upper Left Binary", WINDOW_AUTOSIZE);
    namedWindow("Upper Right Binary", WINDOW_AUTOSIZE);
    namedWindow("Lower Binary", WINDOW_AUTOSIZE);
    namedWindow("Combined Result", WINDOW_AUTOSIZE);

    // Process the image parts
    process_all();

    cout << "Processing complete! Press 'q' or ESC to exit.\n";

    // Wait for user to exit
    while (true) {
        int key = waitKey(1);
        if (key == 'q' || key == 27) break;
    }

    imwrite("rectangles_combined.jpg", result_img);
    destroyAllWindows();
    return 0;
}
