#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
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

    std::cout << "\n[" << naziv << "] Raspon po osima:" << std::endl;
    std::cout << "  X: [" << min_x << ", " << max_x << "]" << std::endl;
    std::cout << "  Y: [" << min_y << ", " << max_y << "]" << std::endl;
    std::cout << "  Z: [" << min_z << ", " << max_z << "]" << std::endl;
}

int main() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cleaned_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::string input_file = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/01_3D vision and point cloud data/pcl_ws/data/rezultati/generated_cloud_1_ptf.pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(input_file, *input_cloud) == -1) {
        PCL_ERROR("Greška pri učitavanju filtrirane Z datoteke!\n");
        return -1;
    }

    std::cout << "Učitano točaka: " << input_cloud->points.size() << std::endl;
    print_bounds(input_cloud, "Prije SOR filtriranja");

    // Primijeni Statistical Outlier Removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(50);               // broj susjeda za računanje prosjeka
    sor.setStddevMulThresh(1.0);    // prag standardne devijacije
    sor.filter(*cleaned_cloud);

    std::cout << "\nNakon SOR filtriranja: " << cleaned_cloud->points.size() << " točaka." << std::endl;
    print_bounds(cleaned_cloud, "Nakon SOR filtriranja");

    std::string output_file = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/01_3D vision and point cloud data/pcl_ws/data/rezultati/cloud_1_sop.pcd";
    if (pcl::io::savePCDFileASCII(output_file, *cleaned_cloud) == -1) {
        PCL_ERROR("Greška pri spremanju SOR rezultata!\n");
        return -1;
    }

    std::cout << "SOR rezultat spremljen u: " << output_file << std::endl;

    return 0;
}
