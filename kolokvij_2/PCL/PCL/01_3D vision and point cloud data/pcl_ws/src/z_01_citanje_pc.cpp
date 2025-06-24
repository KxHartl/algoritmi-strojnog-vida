#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <cfloat>

int main() {
    // 2. Stvori point cloud objekt imena cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


    // 3. Učitaj .pcd datoteku
    std::string filename = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/01_3D vision and point cloud data/pcl_ws/data/primjer/cloud_1.pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
        PCL_ERROR("Greška pri učitavanju datoteke!\n");
        return -1;
    }

    // 4. Ispis osnovnih informacija
    std::cout << "Učitan point cloud: " << filename << std::endl;
    std::cout << "Broj točaka: " << cloud->points.size() << std::endl;
    

    return 0;
}
