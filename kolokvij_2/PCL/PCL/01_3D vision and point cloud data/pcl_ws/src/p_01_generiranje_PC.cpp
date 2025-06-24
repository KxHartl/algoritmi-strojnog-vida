#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

int main() {
    // 1. Kreiranje praznog point clouda
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 2. Definiranje veličine
    cloud->width = 500;
    (*cloud).height = 5;
    cloud->points.resize(cloud->width * cloud->height);

    // 3. Generiranje nasumičnih točaka
    for (auto& point : cloud->points) {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    std::cout << "Generirano " << cloud->points.size() << " točaka." << std::endl;

    // 4. Spremanje u data folder
    std::string output_path = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/01_3D vision and point cloud data/pcl_ws/data/rezultati/generated_cloud.pcd";
    pcl::io::savePCDFileASCII(output_path, *cloud); 
    std::cout << "Point cloud spremljen u '" << output_path << "'." << std::endl;

    return 0;
}
