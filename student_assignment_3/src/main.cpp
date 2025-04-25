#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

int main() {
    // Print OpenCV version
    // cout << "OpenCV version: " << CV_VERSION << endl;
    // Create a simple image
    // Mat image = Mat::zeros(300, 300, CV_8UC3);
    // Draw a red circle
    // circle(image, cv::Point(150, 150), 50, Scalar(0, 0, 255), -1);
    

    // Mat image = imread("../data/screws.jpg",1);
    // image=image;

    // Zad 2;
    // Mat image2= image.clone();
    // cvtColor(image,image2,COLOR_RGB2GRAY);
    // resize(image2,image2,Size(),0.5,0.5,INTER_LINEAR);

    // Zad 3;
    // Mat image2=image.clone();
    // add(image,image2,image2);
    // image2.convertTo(image,-1,1.5,30);

    // zad 4;
    // Rect myROI(50,50,100,100);
    // Mat image_ROI = image(myROI).clone();

    // cout<<image.empty()<<endl;
    // cout<<image.size()<<endl;
    // cout<<image.channels()<<endl;
    // Show the image

    //  Zad 4;
    // Mat image = Mat::zeros(Size(200, 200), CV_8U);
    // circle(image, Point(100,100),100,Scalar(255),-1,LINE_8,0);


    // Mat image2 = Mat::zeros(Size(200, 200), CV_8U);
    // rectangle(image2, Rect(20,20,160,160),Scalar(255),-1,LINE_8,0);
    
    // Mat result1, result2, result3;
    // bitwise_and(image,image2,result1);
    // bitwise_or(image,image2,result2);
    // bitwise_xor(image,image2,result3);

    // hconcat(result1, result2, result3);
    // vconcat(result1, result2, result3);

    // imshow("OpenCV Test", image);
    // imshow("OpenCV Test2", image2);
    // imshow("OpenCV Test3", result1);
    // imshow("OpenCV Test4", result2);
    // imshow("OpenCV Test5", result3);
    // imshow("OpenCV Test6", result3);

    // Zad 1;
    // Mat image = imread("../data/parts.png",1);
    // resize(image,image,Size(),0.5,0.5,INTER_LINEAR);
    // Mat image2;
    // Mat image3;
    // Mat image4;
    // blur(image, image2, Size(15,15));
    // GaussianBlur(image, image3, Size(5,5),0,0);
    // medianBlur(image, image4, 7);

    // imshow("OpenCV Test", image);
    // imshow("OpenCV Test2", image2);
    // imshow("OpenCV Test3", image3);
    // imshow("OpenCV Test4", image4);

    // Zad 2;


    Mat image = imread("../data/parts.png",1);
    resize(image,image,Size(),0.5,0.5,INTER_LINEAR);

    Mat gaussian_noise = Mat (image.size(),image.type());
    randn(gaussian_noise,50,10);
    add(image,gaussian_noise,image);

    Mat image2;
    Mat image3;
    Mat image4;

    
    Mat saltpepper_noise = Mat::zeros(image.rows, image.cols,CV_8U);
    randu(saltpepper_noise,0,255);
    Mat black = saltpepper_noise < 30;
    Mat white = saltpepper_noise > 225;
    image.setTo(255,white);
    image.setTo(0,black);

    blur(image, image2, Size(15,15));
    GaussianBlur(image, image3, Size(5,5),0,0);
    medianBlur(image, image4, 7);

    imshow("OpenCV Test", image);
    imshow("OpenCV Test2", image2);
    imshow("OpenCV Test3", image3);
    imshow("OpenCV Test4", image4);



    waitKey(0); // Wait for key press
    return 0;
}