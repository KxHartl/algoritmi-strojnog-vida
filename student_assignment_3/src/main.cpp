#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

int main() {
    Mat image = imread("../data/parts.png", 0);
    resize(image,image,Size(),0.5,0.5,INTER_LINEAR);

    Mat image_gausian_noise = image.clone();
    Mat image_salt_and_papar_noise = image.clone();

    // Gaussian noise
    Mat gaussian_noise = Mat (image.size(),image.type());
    randn(gaussian_noise,50,10);
    add(image, gaussian_noise, image_gausian_noise);
    
    // Salt and paper noise
    Mat saltpepper_noise = Mat::zeros(image.rows, image.cols,CV_8U);
    randu(saltpepper_noise,0,255);
    Mat black = saltpepper_noise < 10;
    Mat white = saltpepper_noise > 245;
    image_salt_and_papar_noise.setTo(255,white);
    image_salt_and_papar_noise.setTo(0,black);

    imshow("Original", image);
    imshow("Gaussian Noise", image_gausian_noise);
    imshow("Salt and paper", image_salt_and_papar_noise);

    waitKey(0); // Wait for key press
    return 0;
}