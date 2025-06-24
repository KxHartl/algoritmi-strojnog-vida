#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>

int main() {
    // Učitaj point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::string input_file = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL- zad_2_1/results/08_cloud_2.pcd";    
    std::string output_file = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL- zad_2_1/results/09_cloud_2.pcd"; 

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
    seg.setDistanceThreshold(0.01); 
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coeff);

    if (inliers->indices.empty()) {
        std::cerr << "Ravnina nije pronađena!" << std::endl;
        return -1;
    }

    std::cout << "Koeficijenti ravnine: ";
    for (float c : coeff->values) std::cout << c << " ";
    std::cout << std::endl;

    float a = coeff->values[0];
    float b = coeff->values[1];
    float c = coeff->values[2];
    float d = coeff->values[3];

    // Zadrži samo točke koje su iznad ili na ravnini
    for (const auto& pt : cloud->points) {
        float distance = a * pt.x + b * pt.y + c * pt.z + d;
        if (distance >= 0) { // >= znači uključuje ravninu
            filtered->points.push_back(pt);
        }
    }

    filtered->width = filtered->points.size();
    filtered->height = 1;
    filtered->is_dense = true;

    if (pcl::io::savePCDFileASCII(output_file, *filtered) == -1) {
        std::cerr << "Greška pri spremanju: " << output_file << std::endl;
        return -1;
    }

    std::cout << "Spremljeno: " << output_file << " (" << filtered->points.size() << " točaka)" << std::endl;
    return 0;
}
