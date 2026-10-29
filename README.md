# Machine vision algorithms: OpenCV and PCL exercises in C++

**Algoritmi strojnog vida**

> Course exercises – Machine Vision Algorithms (Algoritmi strojnog vida), FSB, University of Zagreb, 2025

![C++](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?logo=opencv&logoColor=white)
![PCL](https://img.shields.io/badge/PCL-point%20clouds-2E8B57)
![CMake](https://img.shields.io/badge/CMake-064F8C?logo=cmake&logoColor=white)

Lab exercises, assignments and midterm preparation for the Machine Vision Algorithms course,
written in C++. The 2D part uses OpenCV for filtering, thresholding, contours, ArUco markers and
feature matching. The 3D part uses the Point Cloud Library for filtering, segmentation and
registration of point clouds.

## What's inside

### Lab exercises

| Folder | Topic |
|---|---|
| `vjezbe_1` | C++ basics |
| `vjezbe_2` | First OpenCV programs: loading images, regions of interest, drawing |
| `vjezbe_3` | Basic filters and morphology |
| `vjezbe_4` | Thresholding |
| `vjezbe_5` | Object and shape localisation |
| `vjezbe_6` | Template and feature matching |
| `feature_matching` | Exercise material for feature matching with ORB and for ArUco perspective correction |

### Assignments

| Folder | What it does |
|---|---|
| `student_assignment_1` | Binary search on a sorted vector in plain C++ |
| `student_assignment_3` | Smoothing filters on noisy images with trackbars, and corner detection with the hit-or-miss transform |
| `student_assignment_4` | Adaptive thresholding with parameters set from trackbars |
| `student_assignment_5` | Contour detection, polygon approximation and a vertex count drawn on each shape |
| `student_assignment_6` | ArUco marker detection, perspective rectification with a homography, then contour detection on the rectified image |

### Midterm preparation

| Folder | What it does |
|---|---|
| `kolokvij_1` | Filtering, adaptive thresholding and contour detection per image region, and a C++ recap (`summary.cpp`) |
| `kolokvij_2/OpenCV3` | Object detection with ORB and SURF feature matching and a homography |
| `kolokvij_2/PCL` | Point cloud generation and filtering (pass-through, statistical outlier removal, voxel grid), registration with K-FPCS, and a case study with RANSAC plane fitting, clustering, transformation and ICP on recorded scenes |

## How to build

Each folder is a separate CMake project. OpenCV is required, and PCL for the point cloud
projects. For example:

```bash
cd student_assignment_6
mkdir -p build && cd build
cmake ..
make
./Main
```

`kolokvij_2/OpenCV3/OpenCV3/src/zadatak3.cpp` uses SURF, which needs OpenCV built with the contrib
`xfeatures2d` module.
