#include <pcl/common/common.h> // For pcl::getMinMax3D
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/pcd_io.h>

void computeBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // Variables to store min and max points
    pcl::PointXYZ min_point, max_point;

    // Compute the bounding box
    pcl::getMinMax3D(*cloud, min_point, max_point);

    // Print the bounding box dimensions
    std::cout << "Bounding Box:" << std::endl;
    std::cout << "  Min Point: (" << min_point.x << ", " << min_point.y << ", " << min_point.z << ")" << std::endl;
    std::cout << "  Max Point: (" << max_point.x << ", " << max_point.y << ", " << max_point.z << ")" << std::endl;
    std::cout << "  Width (X): " << max_point.x - min_point.x << std::endl;
    std::cout << "  Height (Y): " << max_point.y - min_point.y << std::endl;
    std::cout << "  Depth (Z): " << max_point.z - min_point.z << std::endl;
}

int main() {
    // Load your point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("axial_downsampled_100.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file bunny_downsampled.pcd\n");
        return -1;
    }

    // Print the size of the point cloud
    std::cout << "Number of points: " << cloud->points.size() << std::endl;

    // Compute and display the bounding box
    computeBoundingBox(cloud);

    return 0;
}
