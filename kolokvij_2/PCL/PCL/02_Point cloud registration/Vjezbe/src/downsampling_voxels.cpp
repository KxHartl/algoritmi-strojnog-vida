#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>

int main(int argc, char** argv) {
    // Check for correct usage
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <file1.pcd> <file2.pcd>" << std::endl;
        return -1;
    }

    // Create PointCloud objects for the input clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());

    // Load the first PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud1) == -1) {
        std::cerr << "Failed to load PCD file: " << argv[1] << std::endl;
        return -1;
    }

    // // Load the second PCD file
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *cloud2) == -1) {
    //     std::cerr << "Failed to load PCD file: " << argv[2] << std::endl;
    //     return -1;
    // }

    // Print initial point counts
    std::cout << "Loaded " << cloud1->width * cloud1->height
              << " points from " << argv[1] << " (cloud1)." << std::endl;
    std::cout << "Loaded " << cloud2->width * cloud2->height
              << " points from " << argv[2] << " (cloud2)." << std::endl;

    // Downsample both point clouds using VoxelGrid
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    float leafSize = 0.0026;//Adjust for desired downsampling resolution

    // Downsample cloud1
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    voxelGrid.setInputCloud(cloud1);
    voxelGrid.setLeafSize(leafSize, leafSize, leafSize);
    voxelGrid.filter(*cloud1_downsampled);
    std::cout << "Downsampled cloud1 to " << cloud1_downsampled->width * cloud1_downsampled->height
              << " points." << std::endl;

    // // Downsample cloud2
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    // voxelGrid.setInputCloud(cloud2);
    // voxelGrid.setLeafSize(leafSize, leafSize, leafSize);
    // voxelGrid.filter(*cloud2_downsampled);
    // std::cout << "Downsampled cloud2 to " << cloud2_downsampled->width * cloud2_downsampled->height
    //           << " points." << std::endl;

    // Save the downsampled point clouds to new files
    
    std::string cloud1_output = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe/data/01zekan_glava_downsampled.pcd";
    // std::string cloud2_output = "axial_downsampled_100_partial_60_full_transformation_downsampled.pcd";

    pcl::io::savePCDFileBinary(cloud1_output, *cloud1_downsampled);
    // pcl::io::savePCDFileBinary(cloud2_output, *cloud2_downsampled);

    std::cout << "Saved downsampled cloud1 to " << cloud1_output << std::endl;
    // std::cout << "Saved downsampled cloud2 to " << cloud2_output << std::endl;

    return 0;
}
