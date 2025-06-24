#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <cfloat>

void print_bounds(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& naziv) {
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::string input_file = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL/results/05_cloud_4.pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(input_file, *input_cloud) == -1) {
        PCL_ERROR("Greška pri učitavanju datoteke!\n");
        return -1;
    }

    std::cout << "Učitano točaka: " << input_cloud->points.size() << std::endl;
    print_bounds(input_cloud, "Prije VoxelGrid-a");

    // Primjena VoxelGrid filtra (vokselizacija)
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(input_cloud);
    voxel.setLeafSize(0.005f, 0.005f, 0.005f);  // postavi veličinu voxela (1 cm)
    voxel.filter(*downsampled_cloud);

    std::cout << "\nNakon VoxelGrid filtriranja: " << downsampled_cloud->points.size() << " točaka." << std::endl;
    print_bounds(downsampled_cloud, "Nakon VoxelGrid-a");

    std::string output_file = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL/results/06_cloud_4.pcd";
    if (pcl::io::savePCDFileASCII(output_file, *downsampled_cloud) == -1) {
        PCL_ERROR("Greška pri spremanju vokseliziranog oblaka!\n");
        return -1;
    }

    std::cout << "Vokselizirani oblak spremljen u: " << output_file << std::endl;

    return 0;
}
