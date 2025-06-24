
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>
#include <unordered_set>

int main() {
    // Učitavanje oblaka
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::string input_file = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL- zad_2_1/results/07_cloud_2.pcd";   
    std::string output_file = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL- zad_2_1/results/08_cloud_2.pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(input_file, *cloud) == -1) {
        std::cerr << "Greška pri učitavanju: " << input_file << std::endl;
        return -1;
    }

    cloud->is_dense = true;
    std::cout << "Učitano točaka: " << cloud->points.size() << std::endl;

    // Segmentacija ravnine
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01); // 1 cm tolerancije
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coeff);

    if (inliers->indices.empty()) {
        std::cerr << "Ravnina nije pronađena!" << std::endl;
        return -1;
    }

    std::cout << "Detektirano točaka u ravnini: " << inliers->indices.size() << std::endl;

    // Filtriraj točke koje nisu dio ravnine
    std::unordered_set<int> inlier_set(inliers->indices.begin(), inliers->indices.end());

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        if (inlier_set.count(static_cast<int>(i)) == 0) {
            filtered->points.push_back(cloud->points[i]);
        }
    }

    filtered->width = filtered->points.size();
    filtered->height = 1;
    filtered->is_dense = true;

    // Spremi rezultat
    if (pcl::io::savePCDFileASCII(output_file, *filtered) == -1) {
        std::cerr << "Greška pri spremanju: " << output_file << std::endl;
        return -1;
    }

    std::cout << "Spremljeno: " << output_file << " (" << filtered->points.size() << " točaka)" << std::endl;
    return 0;
}
