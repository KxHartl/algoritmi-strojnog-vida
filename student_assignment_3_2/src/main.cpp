#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

int main() {
     // Step 1: Create image matrix with rectangles
     Mat image = cv::Mat::zeros(27, 27, CV_8UC1);
     rectangle(image, Rect(3, 3, 12, 9), Scalar(255), -1);
     rectangle(image, Rect(21, 3, 3, 3), Scalar(255), -1);
     rectangle(image, Rect(6, 15, 3, 3), Scalar(255), -1);
     rectangle(image, Rect(3, 18, 3, 6), Scalar(255), -1);
     rectangle(image, Rect(6, 21, 3, 3), Scalar(255), -1);
     rectangle(image, Rect(9, 18, 3, 6), Scalar(255), -1);
     rectangle(image, Rect(15, 15, 6, 3), Scalar(255), -1);
     rectangle(image, Rect(18, 18, 3, 3), Scalar(255), -1);

     

     resize(image, image, Size(), 8, 8, INTER_NEAREST);
     imshow("Original", image);
     
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
