#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>

// Učitavanje 4x4 matrice iz .txt datoteke
bool loadMatrixFromTxt(const std::string& filename, Eigen::Matrix4f& mat) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Greška: ne mogu otvoriti " << filename << std::endl;
        return false;
    }

    for (int i = 0; i < 4 && !file.eof(); ++i) {
        for (int j = 0; j < 4 && !file.eof(); ++j) {
            file >> mat(i, j);
        }
    }
    file.close();
    return true;
}

int main() {
    // Učitaj point cloud 1 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile("/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL- zad_2/data/04_2_cloud_2.pcd", *cloud1) == -1) {
        std::cerr << "Ne mogu učitati cloud_1.pcd" << std::endl;
        return -1;
    }

    // Učitaj point cloud 2
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile("/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL- zad_2/data/05_2_cloud_5.pcd", *cloud2) == -1) {
        std::cerr << "Ne mogu učitati cloud_2.pcd" << std::endl;
        return -1;
    }

    // Učitaj transformacijske matrice
    Eigen::Matrix4f T_base_tcp_1, T_base_tcp_2, T_tcp_cam;

    if (!loadMatrixFromTxt("/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL- zad_2/data/cloud_2.txt", T_base_tcp_1)) return -1;
    if (!loadMatrixFromTxt("/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL- zad_2/data/cloud_5.txt", T_base_tcp_2)) return -1;
    if (!loadMatrixFromTxt("/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL- zad_2/data/D405 ekstrinzicna.txt", T_tcp_cam)) return -1;

    // Konačne transformacije za oba oblaka 
    Eigen::Matrix4f T_base_cam_1 = T_base_tcp_1 * T_tcp_cam;
    Eigen::Matrix4f T_base_cam_2 = T_base_tcp_2 * T_tcp_cam;

    std::cout << "T_base_cam_1:\n" << T_base_cam_1 << std::endl;
    std::cout << "T_base_cam_2:\n" << T_base_cam_2 << std::endl;    

    // Transformiraj point cloudove 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_trans(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud1, *cloud1_trans, T_base_cam_1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2_trans(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud2, *cloud2_trans, T_base_cam_2);

    // Spremi rezultate 
    pcl::io::savePCDFileBinary("/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL- zad_2/data/11_2_cloud_2.pcd", *cloud1_trans);
    pcl::io::savePCDFileBinary("/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/03_Case study - PCL- zad_2/data/11_2_cloud_5.pcd", *cloud2_trans);

    std::cout << "Transformacija gotova. Spremljeno kao transformed_cloud_1.pcd i transformed_cloud_2.pcd" << std::endl;
    return 0;
}
