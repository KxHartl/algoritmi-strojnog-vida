#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <iostream>

int main() {
    using PointT = pcl::PointXYZRGB;

    // === 1. Učitaj transformirane point cloudove ===
    pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile("/home/asvtara/pcl_ws_prof/data/asv_dataset/scene_01_obradeno/transformed_cloud_1.pcd", *cloud1) == -1 ||
        pcl::io::loadPCDFile("/home/asvtara/pcl_ws_prof/data/asv_dataset/scene_01_obradeno/transformed_cloud_2.pcd", *cloud2) == -1) {
        std::cerr << "Greška pri učitavanju .pcd datoteka." << std::endl;
        return -1;
    }

    // === 2. Ručna korekcija transformacije ===
    Eigen::Matrix4f manual_adjustment = Eigen::Matrix4f::Identity();
    manual_adjustment(0, 3) = 0.01f; // translacija u X (1 cm)
    manual_adjustment(1, 3) = 0.00f; // translacija u Y
    manual_adjustment(2, 3) = -0.01f; // translacija u Z
    Eigen::AngleAxisf rotx(0.0873, Eigen::Vector3f::UnitX()); // 5 deg u radijanima
    Eigen::Matrix4f pre_rot = Eigen::Matrix4f::Identity();
    pre_rot.block<3,3>(0,0) = rotx.toRotationMatrix();

    manual_adjustment = pre_rot * manual_adjustment;

    pcl::PointCloud<PointT>::Ptr cloud2_adjusted(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cloud2, *cloud2_adjusted, manual_adjustment);

    // === 3. Postavi i pokreni ICP ===
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(cloud2_adjusted);
    icp.setInputTarget(cloud1);
    icp.setMaximumIterations(100);
    icp.setMaxCorrespondenceDistance(0.05); // 5 cm max udaljenost za match
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1);

    pcl::PointCloud<PointT>::Ptr cloud_aligned(new pcl::PointCloud<PointT>);
    icp.align(*cloud_aligned);

    if (icp.hasConverged()) {
        std::cout << "ICP konvergirao!" << std::endl;
        std::cout << "Fitness score: " << icp.getFitnessScore() << std::endl;

        // === 4. Spremi poravnati cloud ===
        pcl::io::savePCDFileBinary("/home/asvtara/pcl_ws_prof/data/asv_dataset/scene_01_obradeno/cloud_2_icp_aligned.pcd", *cloud_aligned);

        // === 5. Spoji s cloud_1 ===
        *cloud1 += *cloud_aligned;
        pcl::io::savePCDFileBinary("/home/asvtara/pcl_ws_prof/data/asv_dataset/scene_01_obradeno/merged_icp_cloud.pcd", *cloud1);

        std::cout << "Spojeni oblak spremljen kao merged_icp_cloud.pcd" << std::endl;
    } else {
        std::cerr << "ICP nije konvergirao!" << std::endl;
        return -1;
    }

    return 0;
}
