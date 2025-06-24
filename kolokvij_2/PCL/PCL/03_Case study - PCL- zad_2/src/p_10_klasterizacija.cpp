#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <iostream>
#include <vector>

int main() {
    using PointT = pcl::PointXYZRGB;

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    std::string input_file = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL/results/08_cloud_4.pcd";

    if (pcl::io::loadPCDFile<PointT>(input_file, *cloud) == -1) {
        PCL_ERROR("Greška pri učitavanju point clouda!\n");
        return -1;
    }

    std::cout << "Učitano točaka: " << cloud->points.size() << std::endl;

    // KdTree za pretraživanje susjeda
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.02); // 2 cm
    ec.setMinClusterSize(500);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    std::cout << "Broj pronađenih klastera: " << cluster_indices.size() << std::endl;

    // Spremanje svakog klastera kao poseban .pcd
    int j = 0;
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        for (int idx : indices.indices)
            cluster->points.push_back(cloud->points[idx]);
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        std::string filename = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL/results/10_cloud_4_" + std::to_string(j+1) + ".pcd";
        pcl::io::savePCDFileASCII(filename, *cluster);
        std::cout << "Spremljen klaster #" << j << " s " << cluster->points.size() << " točaka u: " << filename << std::endl;
        j++;
    }

    return 0;
}
