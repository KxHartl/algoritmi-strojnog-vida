#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <iostream>
#include <cfloat>  // za FLT_MAX

// Funkcija za ispis min/max koordinata po svakoj osi
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::string filename = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL- zad_2_1/data/cloud_2.pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *original_cloud) == -1) {
        PCL_ERROR("Greška pri učitavanju datoteke!\n");
        return -1;
    }

    std::cout << "Učitano točaka: " << original_cloud->points.size() << std::endl;
    print_bounds(original_cloud, "Prije filtriranja");

    // PassThrough po Z osi
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(original_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.1, 0.44);  // promijeni po potrebi
    pass.filter(*filtered_cloud);

    std::cout << "\nFiltrirano točaka: " << filtered_cloud->points.size() << std::endl;
    print_bounds(filtered_cloud, "Nakon filtriranja");

    std::string output_file = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL- zad_2_1/results/04_cloud_2.pcd";
    if (pcl::io::savePCDFileASCII(output_file, *filtered_cloud) == -1) {
        PCL_ERROR("Greška pri spremanju datoteke!\n");
        return -1;
    }

    std::cout << "Rezultat spremljen u: " << output_file << std::endl;

    return 0;
}
