// color_plane.cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>

int main() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);


    std::string input_file = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL- zad_2_1/results/04_cloud_2.pcd";    
    std::string output_file = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL- zad_2_1/results/07_cloud_2.pcd"; 

    if (pcl::io::loadPCDFile(input_file, *cloud) == -1) {
        std::cerr << "Greška pri učitavanju cloud_input.pcd" << std::endl;
        return -1;
    }  

    cloud->is_dense = true;

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coeff);

    std::cout << "Točaka u ravnini: " << inliers->indices.size() << std::endl;

    for (int idx : inliers->indices) {
        cloud->points[idx].r = 255;
        cloud->points[idx].g = 0;
        cloud->points[idx].b = 0;
    }

    pcl::io::savePCDFileASCII(output_file , *cloud);

    std::cout << "Spremljeno obojano" << std::endl;
    return 0;
}
