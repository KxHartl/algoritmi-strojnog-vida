#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <iostream>
using namespace cv;
using namespace std;

int main() {

    /*Excercise 5- Practical task 6

    Mat color;
    Mat image = imread("../data/lines.png",0);
    Canny(image,image,150 ,260 ,3);

    imshow("CANNY", image);


    cvtColor(image, color, COLOR_GRAY2BGR);

    vector <Vec2f> lines; //ovo znaci da je vektor ali array

    // HoughLines(image, lines, 1, CV_PI/180, 185, 0 ,0);


    // // Draw the lines
    // for( size_t i = 0; i < lines.size(); i++ )
    // {
    // float rho = lines[i][0], theta = lines[i][1];
    // Point pt1, pt2;
    // double a = cos(theta), b = sin(theta);
    // double x0 = a*rho, y0 = b*rho;
    // pt1.x = cvRound(x0 + 1000*(-b));
    // pt1.y = cvRound(y0 + 1000*(a));
    // pt2.x = cvRound(x0 - 1000*(-b));
    // pt2.y = cvRound(y0 - 1000*(a));
    // line( color, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
    // }

        // Probabilistic Line Transform
    vector<Vec4i> linesP; // will hold the results of the detection
    HoughLinesP(image, linesP, 1, CV_PI/180, 50, 50 , 1 ); // runs the actual detection
    // Draw the lines
    for( size_t i = 0; i < linesP.size(); i++ )
    {
    Vec4i l = linesP[i];
    line( color, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
    }


    imshow("original", color);
    */




    //Excercise 5- Practical task 7
    /*
    Mat color = imread("../data/circles.png",1);
    Mat image = imread("../data/circles.png",0);

    blur(image, image, Size(3,3));


    imshow("image", image);


    vector <Vec3f> circles;
    HoughCircles(image, circles, HOUGH_GRADIENT, 1,
        image.rows/16, // change this value to detect circles with different distances to each other
        80, 30, 20, 30 // change the last two parameters
        // (min_radius & max_radius) to detect larger circles
        );

    for( size_t i = 0; i < circles.size(); i++ )
    {
    Vec3i c = circles[i];
    Point center = Point(c[0], c[1]); // circle center
    circle( color, center, 1, Scalar(0,100,100), 3, LINE_AA); // circle outline
    int radius = c[2];
    circle( color, center, radius, Scalar(255,0,255), 3, LINE_AA);
    }

    cvtColor(image, color, COLOR_GRAY2BGR);

    vector <Vec2f> lines; //ovo znaci da je vektor ali array
    
    imshow("image s kruznicama", color);
    
    */
/*
    //Excercise 6- Practical task 1

    Mat scena = imread("../data/scena.png",0);
    Mat temp1 = imread("../data/plocica.png",0);
    Mat prikaz = imread("../data/scena.png",1);
    Mat result;

    matchTemplate(scena,temp1,result, 1);
    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;
    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
    rectangle(prikaz, minLoc, Point(minLoc.x + temp1.cols, minLoc.y + temp1.rows), Scalar(0,0,255), 2,8,0);



    imshow("rezultat",prikaz);
    */
   // Excercise 6- Practical task 2,3

    // Load the predefined 4x4 ArUco dictionary
    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

    // Generate an ArUco marker image from the dictionary
    Mat markerImage;
    int markerId = 5;
    int markerSizePixels = 200;
    aruco::generateImageMarker(dictionary, markerId, markerSizePixels, markerImage);

    // Display and save the generated marker
    imshow("Generated ArUco Marker", markerImage);
    imwrite("..data/aruco_marker5.png", markerImage);

    // Create a larger colored background image
    int bgSize = 800;
    Mat bg(bgSize, bgSize, CV_8UC3, Scalar(200, 50, 100)); // light blue background

    // Convert the marker to a 3-channel image and place it at the center
    Mat markerColor;
    cvtColor(markerImage, markerColor, COLOR_GRAY2BGR);
    int xOffset = (bg.cols - markerColor.cols) / 2;
    int yOffset = (bg.rows - markerColor.rows) / 2;
    markerColor.copyTo(bg(Rect(xOffset, yOffset, markerColor.cols, markerColor.rows)));

    imshow("Marker on Background", bg);

    // Apply a 3D rotation using a perspective warp to simulate marker pose
    // Define a camera intrinsic matrix (focal lengths and optical center)
    Mat K = (Mat_<double>(3,3) << bg.cols/2, 0, bg.cols/2, 0, bg.rows/2, bg.rows/2, 0, 0, 1);

    // Define a 3D rotation vector in radians
    Mat rvec = (Mat_<double>(3,1) << 20 * CV_PI/180, 20 * CV_PI/180, 20 * CV_PI/180);
    Mat tvec = Mat::zeros(3,1,CV_64F); // No translation

    // Convert rotation vector to rotation matrix
    Mat R;
    Rodrigues(rvec, R);

    // Compute homography for simulating the 3D rotation
    Mat H = K * R * K.inv();

    // Warp the background image using the homography
    Mat warped;
    warpPerspective(bg, warped, H, bg.size());
    imshow("Warped Image", warped);

    // Detect ArUco markers in the warped image
    aruco::ArucoDetector detector(dictionary);
    vector<int> ids;
    vector<vector<Point2f>> corners;
    detector.detectMarkers(warped, corners, ids);

    if (!ids.empty()) {
        aruco::drawDetectedMarkers(warped, corners, ids);

        // Estimate pose of the detected marker (solvePnP)
        vector<Point3f> objPoints = { {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0} }; // Marker corner points in 3D
        Mat distCoeffs = Mat::zeros(5,1,CV_64F); // No lens distortion
        Vec3d rvecEst, tvecEst;
        solvePnP(objPoints, corners[0], K, distCoeffs, rvecEst, tvecEst);

        // Define and project a cube to simulate 3D augmentation
        vector<Point3f> cubePoints = {
            {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0},          // Bottom square
            {0,0,-1}, {1,0,-1}, {1,1,-1}, {0,1,-1}       // Top square
        };
        vector<Point2f> projectedPoints;
        projectPoints(cubePoints, rvecEst, tvecEst, K, distCoeffs, projectedPoints);

        // Define the cube edges and draw lines between projected 2D points
        int edges[][2] = {{0,1},{1,2},{2,3},{3,0},       // Bottom face
                            {4,5},{5,6},{6,7},{7,4},       // Top face
                            {0,4},{1,5},{2,6},{3,7}};      // Vertical edges

        for (auto& e : edges) {
            line(warped, projectedPoints[e[0]], projectedPoints[e[1]], Scalar(0,255,0), 2);
        }
    } else {
        cout << "No marker detected!" << endl;
    }

    // Display the final image with marker detection and cube
    imshow("Detected with Cube", warped);




    waitKey(0); // Wait for key press
    return 0;
}