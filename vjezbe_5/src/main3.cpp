#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(){
    Mat image = imread("../data/lines.png", 0);
    Mat color;

    Canny(image, image, 150, 500, 3);
    cvtColor(image, color, COLOR_GRAY2BGR);

    // HoughLines
    // 
    // vector <Vec2f> lines;
    // HoughLines(image, lines, 1, CV_PI/100, 120, 0, 0);
    // for( size_t i = 0; i < lines.size(); i++ )
    //     {
    //     float rho = lines[i][0], theta = lines[i][1];
    //     Point pt1, pt2;
    //     double a = cos(theta), b = sin(theta);
    //     double x0 = a*rho, y0 = b*rho;
    //     pt1.x = cvRound(x0 + 1000*(-b));
    //     pt1.y = cvRound(y0 + 1000*(a));
    //     pt2.x = cvRound(x0 - 1000*(-b));
    //     pt2.y = cvRound(y0 - 1000*(a));
    //     line(color, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
    //     }

    // Probabilistic Line Transform
    // 
    vector<Vec4i> linesP; // will hold the results of the detection
    HoughLinesP(image, linesP, 1, CV_PI/180, 20, 40, 1 ); // runs the actual detection
    // Draw the lines
    for( size_t i = 0; i < linesP.size(); i++ )
        {
        Vec4i l = linesP[i];
        line(color, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
        }

    
    imshow("Color", color);
    imshow("Canny", image);

    while (true) {
        int key = waitKey(1); // Wait for 1 ms for any key press
        if (key == 'q' || key == 27) { // Exit on 'q' or ESC key
            break;
        }
    }
    destroyAllWindows();
    return 1;
    }