#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Korak 1: Kreiranje slike s presječenim oblicima
    cv::Mat image = cv::Mat::zeros(300, 300, CV_8UC1);
    
    // Crtanje pravokutnika
    cv::rectangle(image, cv::Rect(50, 50, 200, 100), cv::Scalar(255), -1);
    
    // Crtanje kruga koji se presijeca s pravokutnikom
    cv::circle(image, cv::Point(200, 150), 80, cv::Scalar(255), -1);
    
    // Prikaz originalne slike
    cv::namedWindow("Original", cv::WINDOW_AUTOSIZE);
    cv::imshow("Original", image);
    
    // Kreiranje strukturnog elementa (kernela)
    int kernelSize = 5;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    
    // Primjena morfoloških operacija
    cv::Mat erosion, dilation, opening, closing, gradient, tophat, blackhat;
    
    // Erozija
    cv::erode(image, erosion, kernel);
    cv::namedWindow("Erosion", cv::WINDOW_AUTOSIZE);
    cv::imshow("Erosion", erosion);
    
    // Dilatacija
    cv::dilate(image, dilation, kernel);
    cv::namedWindow("Dilation", cv::WINDOW_AUTOSIZE);
    cv::imshow("Dilation", dilation);
    
    // Otvaranje (erozija praćena dilatacijom)
    cv::morphologyEx(image, opening, cv::MORPH_OPEN, kernel);
    cv::namedWindow("Opening", cv::WINDOW_AUTOSIZE);
    cv::imshow("Opening", opening);
    
    // Zatvaranje (dilatacija praćena erozijom)
    cv::morphologyEx(image, closing, cv::MORPH_CLOSE, kernel);
    cv::namedWindow("Closing", cv::WINDOW_AUTOSIZE);
    cv::imshow("Closing", closing);
    
    // Morfološki gradijent (dilatacija - erozija)
    cv::morphologyEx(image, gradient, cv::MORPH_GRADIENT, kernel);
    cv::namedWindow("Gradient", cv::WINDOW_AUTOSIZE);
    cv::imshow("Gradient", gradient);
    
    // Top Hat (originalna slika - otvaranje)
    cv::morphologyEx(image, tophat, cv::MORPH_TOPHAT, kernel);
    cv::namedWindow("Top Hat", cv::WINDOW_AUTOSIZE);
    cv::imshow("Top Hat", tophat);
    
    // Black Hat (zatvaranje - originalna slika)
    cv::morphologyEx(image, blackhat, cv::MORPH_BLACKHAT, kernel);
    cv::namedWindow("Black Hat", cv::WINDOW_AUTOSIZE);
    cv::imshow("Black Hat", blackhat);
    
    // Čekanje na tipku za zatvaranje svih prozora
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}
