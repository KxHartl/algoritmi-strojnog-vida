#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

using namespace cv;
using namespace std;

int main() {
    // Load the scene and object images in grayscale
    Mat img_scene = imread("../data/scena.png", IMREAD_GRAYSCALE);
    Mat img_object = imread("../data/podlozka1.png", IMREAD_GRAYSCALE);

    if (img_object.empty() || img_scene.empty()) {
        cerr << "Could not open or find the images!" << endl;
        return -1;
    }

    // Step 1: Initialize ORB detector
    int nfeatures = 2000;
    Ptr<ORB> detector = ORB::create(nfeatures);

    // Detect keypoints and compute descriptors
    vector<KeyPoint> keypoints_object, keypoints_scene;
    Mat descriptors_object, descriptors_scene;
    detector->detectAndCompute(img_object, noArray(), keypoints_object, descriptors_object);
    detector->detectAndCompute(img_scene, noArray(), keypoints_scene, descriptors_scene);

    // Step 2: Create BFMatcher (Hamming distance for ORB)
    Ptr<DescriptorMatcher> matcher = BFMatcher::create(NORM_HAMMING, false);

    // K-NN matching
    vector<vector<DMatch>> knn_matches;
    matcher->knnMatch(descriptors_object, descriptors_scene, knn_matches, 2);

    // Ratio test (Lowe’s ratio test)
    const float ratio_thresh = 0.8f;
    vector<DMatch> good_matches;
    for (const auto& match_pair : knn_matches) {
        if (match_pair.size() == 2 && match_pair[0].distance < ratio_thresh * match_pair[1].distance) {
            good_matches.push_back(match_pair[0]);
        }
    }

    // Draw matches
    Mat img_matches;
    drawMatches(img_object, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches,
                Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    // Homography estimation
    if (good_matches.size() >= 4) {
        vector<Point2f> obj_points, scene_points;
        for (const auto& match : good_matches) {
            obj_points.push_back(keypoints_object[match.queryIdx].pt);
            scene_points.push_back(keypoints_scene[match.trainIdx].pt);
        }

        Mat H = findHomography(obj_points, scene_points, RANSAC, 3.0);

        // Define object corners and project to scene
        vector<Point2f> obj_corners = {
            Point2f(0, 0),
            Point2f(static_cast<float>(img_object.cols), 0),
            Point2f(static_cast<float>(img_object.cols), static_cast<float>(img_object.rows)),
            Point2f(0, static_cast<float>(img_object.rows))
        };
        vector<Point2f> scene_corners(4);
        perspectiveTransform(obj_corners, scene_corners, H);

        // Draw box in the scene
        for (int i = 0; i < 4; ++i) {
            line(img_matches,
                 scene_corners[i] + Point2f((float)img_object.cols, 0),
                 scene_corners[(i + 1) % 4] + Point2f((float)img_object.cols, 0),
                 Scalar(0, 255, 0), 4);
        }
    } else {
        cout << "Not enough good matches to compute homography." << endl;
    }

    // Show result
    imshow("ORB Matches & Object Detection", img_matches);
    waitKey();

    return 0;
}