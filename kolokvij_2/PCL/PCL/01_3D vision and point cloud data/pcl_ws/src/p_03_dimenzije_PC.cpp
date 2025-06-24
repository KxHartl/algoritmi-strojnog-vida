#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <cfloat>  // za FLT_MAX

template<typename PointT>
void print_axis_ranges(typename pcl::PointCloud<PointT>::Ptr cloud, const std::string& naziv) {
    float min_x = FLT_MAX, max_x = -FLT_MAX;
    float min_y = FLT_MAX, max_y = -FLT_MAX;
    float min_z = FLT_MAX, max_z = -FLT_MAX;

    for (const auto& pt : cloud->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
            continue;

        if (pt.x < min_x) min_x = pt.x;
        if (pt.x > max_x) max_x = pt.x;

        if (pt.y < min_y) min_y = pt.y;
        if (pt.y > max_y) max_y = pt.y;

        if (pt.z < min_z) min_z = pt.z;
        if (pt.z > max_z) max_z = pt.z;
    }

    std::cout << "\n[" << naziv << "] Raspon koordinata:" << std::endl;
    std::cout << "  X: [" << min_x << ", " << max_x << "]" << std::endl;
    std::cout << "  Y: [" << min_y << ", " << max_y << "]" << std::endl;
    std::cout << "  Z: [" << min_z << ", " << max_z << "]" << std::endl;
}

int main() {
    std::string filename = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/01_3D vision and point cloud data/pcl_ws/data/scene_01/cloud_01.pcd";

    // Možeš promijeniti PointXYZRGB u PointXYZ ako tvoj oblak nema boju
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile(filename, *cloud) == -1) {
        PCL_ERROR("Greška pri učitavanju PCD datoteke!\n");
        return -1;
    }

    std::cout << "Učitano točaka: " << cloud->points.size() << std::endl;
    print_axis_ranges<pcl::PointXYZRGB>(cloud, "Ulazni oblak");

    return 0;
}
