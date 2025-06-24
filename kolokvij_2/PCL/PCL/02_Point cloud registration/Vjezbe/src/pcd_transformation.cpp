#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

int main() {
    // Load the PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe/data/01zekan_glava_downsampled.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read the file\n");
        return -1;
    }
    std::cout << "Loaded " << cloud->size() << " points from input.pcd" << std::endl;

    // Define the transformation
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // Translation: a units in X, b units in Y, c units in Z
    float a=0.3;
    float b=-0.2;
    float c=0.1;
    transform.translation() << a, b, c;

    // Rotation: 45 degrees around Y-axis and Z-axis
    float theta1 = M_PI/3; // rotation angle 1--- 60 stupnjeva oko X
    float theta2 = M_PI/6; // rotation angle 2--- 30 stupnjeva oko Y
    float theta3 = M_PI/2; // rotation angle 3--- 90 stupnjeva oko Z
    transform.rotate(Eigen::AngleAxisf(theta1, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(theta2, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(theta3, Eigen::Vector3f::UnitZ()));

    // Apply the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    // Save the transformed point cloud
    pcl::io::savePCDFileASCII("/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe/data/01zekan_glava_downsampled_nakoso.pcd", *transformed_cloud);
    std::cout << "Transformed point cloud saved to output.pcd" << std::endl;


    return 0;
}