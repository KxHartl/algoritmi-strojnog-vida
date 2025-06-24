#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>  // DODANO
#include <Eigen/Dense>
#include <iostream>

int main() {
    using PointT = pcl::PointXYZRGB;

    // === 1. Učitaj transformirane point cloudove ===
    pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile("/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL- zad_2/data/11_2_cloud_2.pcd", *cloud1) == -1 ||
        pcl::io::loadPCDFile("/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL- zad_2/data/11_2_cloud_5.pcd", *cloud2) == -1) {
        std::cerr << "Greška pri učitavanju .pcd datoteka." << std::endl;
        return -1;
    }

    // === DODANO: Filtriraj NaN vrijednosti ===
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud1, *cloud1, indices);
    pcl::removeNaNFromPointCloud(*cloud2, *cloud2, indices);
    
    std::cout << "Cloud1 veličina nakon filtriranja: " << cloud1->size() << std::endl;
    std::cout << "Cloud2 veličina nakon filtriranja: " << cloud2->size() << std::endl;

    // === 2. Ručna korekcija transformacije ===
    Eigen::Matrix4f manual_adjustment = Eigen::Matrix4f::Identity();
    manual_adjustment(0, 3) = 0.01f; 
    manual_adjustment(1, 3) = 0.00f; 
    manual_adjustment(2, 3) = -0.01f; 
    Eigen::AngleAxisf rotx(0.0873, Eigen::Vector3f::UnitX()); 
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
    icp.setMaxCorrespondenceDistance(0.05); 
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1);

    pcl::PointCloud<PointT>::Ptr cloud_aligned(new pcl::PointCloud<PointT>);
    icp.align(*cloud_aligned);

    if (icp.hasConverged()) {
        std::cout << "ICP konvergirao!" << std::endl;
        std::cout << "Fitness score: " << icp.getFitnessScore() << std::endl;

        // === 4. Spremi poravnati cloud ===
        pcl::io::savePCDFileBinary("/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL- zad_2/data/12_2_cloud_poravnati.pcd", *cloud_aligned);

        // === 5. Spoji s cloud_1 ===
        *cloud1 += *cloud_aligned;
        pcl::io::savePCDFileBinary("/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL- zad_2/data/12_2_cloud_spojeni.pcd", *cloud1);

        std::cout << "Spojeni oblak spremljen kao 12_2_cloud_spojeni" << std::endl;
    } else {
        std::cerr << "ICP nije konvergirao!" << std::endl;
        return -1;
    }

    return 0;
}
