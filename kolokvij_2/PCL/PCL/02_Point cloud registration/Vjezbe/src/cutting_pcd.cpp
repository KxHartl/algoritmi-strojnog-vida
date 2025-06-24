#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

int main() {
    // Specify the input and output PCD file names
    const std::string input_pcd_file = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe/data/bun000.pcd"; // Replace with your input file name
    const std::string output_pcd_file = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe/data/01zekan_glava.pcd";

    // Load the original point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_part1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_part2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_combined(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_file, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", input_pcd_file.c_str());
        return -1;
    }
    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << input_pcd_file << std::endl;

    // First CropBox filter
    pcl::CropBox<pcl::PointXYZ> box_filter_1;
    box_filter_1.setInputCloud(cloud);

    // Define the first box dimensions
    Eigen::Vector4f min_point_1(-0.1, 0.1, 0, 0); // Example: lower corner of first box
    Eigen::Vector4f max_point_1(-0.045, 0.17, 0.4, 0);   // Example: upper corner of first box
    box_filter_1.setMin(min_point_1);
    box_filter_1.setMax(max_point_1);

    // Apply the first filter
    box_filter_1.filter(*cloud_part1);
    std::cout << "First part contains " << cloud_part1->points.size() << " points." << std::endl;

    // // Second CropBox filter
    // pcl::CropBox<pcl::PointXYZ> box_filter_2;
    // box_filter_2.setInputCloud(cloud);

    // // Define the second box dimensions
    // Eigen::Vector4f min_point_2(-0.05, 0.14, -200.0, 0);  // Example: lower corner of second box
    // Eigen::Vector4f max_point_2(0, 0.98, 200.0, 0);  // Example: upper corner of second box
    // box_filter_2.setMin(min_point_2);
    // box_filter_2.setMax(max_point_2);

    // // Apply the second filter
    // box_filter_2.filter(*cloud_part2);
    // std::cout << "Second part contains " << cloud_part2->points.size() << " points." << std::endl;

    // // Combine the two filtered point clouds
    // *cloud_combined = *cloud_part1 + *cloud_part2; // Concatenate point clouds
    // std::cout << "Combined cloud contains " << cloud_combined->points.size() << " points (before filtering)." << std::endl;

    // Apply VoxelGrid filter to remove duplicates
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud_part1);
    voxel_filter.setLeafSize(0.0001f, 0.0001f, 0.0001f); // Set voxel size (adjust as needed)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_combined(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter.filter(*cloud_filtered_combined);

    std::cout << "Combined cloud contains " << cloud_filtered_combined->points.size() << " points (after filtering)." << std::endl;

    // Save the filtered combined point cloud
    pcl::io::savePCDFileASCII(output_pcd_file, *cloud_filtered_combined);
    std::cout << "Saved combined and filtered point cloud to " << output_pcd_file << std::endl;

    return 0;
}
