#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(){

    Mat scena = imread("../data/scena.png", 0);
    Mat templ = imread("../data/plocica.png", 0);
    Mat prikaz = imread("../data/scena.png", 1);
    Mat results;

    

}
