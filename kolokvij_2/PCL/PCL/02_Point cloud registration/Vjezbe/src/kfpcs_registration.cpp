#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>
#include <pcl/registration/ia_kfpcs.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <boost/thread/thread.hpp>
#include <boost/math/distributions/students_t.hpp>
#include <sys/stat.h>
#include <sys/types.h>
#include <fstream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

using namespace std;

double calculateFrobeniusNorm(const Eigen::Matrix4f& matrix) {
    return matrix.norm();
}

int main(int argc, char** argv)
{
    pcl::console::TicToc time;

    // Define the save directory and parameters
    std::string save_dir = "/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe/tmp";

    int numThreads = 5;
    int numSamples = 300;
    float delta = 0.001f;
    float approxOverlap = 0.8f;
    float lambda = 0.7f;
    
    // Create directory with better error handling
    struct stat info;
    if (stat(save_dir.c_str(), &info) != 0)
    {
        if (mkdir(save_dir.c_str(), 0755) == -1)
        {
            cerr << "Error: Unable to create directory " << save_dir << endl;
            cerr << "Error code: " << strerror(errno) << endl;
            return -1;
        }
        cout << "Created directory: " << save_dir << endl;
    }
    else if (!(info.st_mode & S_IFDIR))
    {
        cerr << "Error: " << save_dir << " exists but is not a directory" << endl;
        return -1;
    }

    // Open log file with better path handling and error checking
    std::string log_filename = save_dir + "/kfpcs_data_armadillo.txt";
    std::ofstream logFile(log_filename.c_str(), std::ios::out | std::ios::trunc);
    
    if (!logFile.is_open())
    {
        cerr << "Error: Unable to open log file: " << log_filename << endl;
        cerr << "Error code: " << strerror(errno) << endl;
        return -1;
    }
    
    cout << "Log file opened successfully: " << log_filename << endl;
    
    // Test write to log file
    logFile << "=== KFPCS Registration Log ===" << endl;
    logFile << "Log created at: " << __DATE__ << " " << __TIME__ << endl;
    logFile.flush(); // Force immediate write

    // Calculate theoretical transformation matrix
    double thetaX = M_PI / 3.0;
    double thetaY = M_PI / 6.0;
    double thetaZ = M_PI / 2.0;

    Eigen::Matrix4f Rx = Eigen::Matrix4f::Identity();
    Rx(1, 1) = cos(thetaX);
    Rx(1, 2) = -sin(thetaX);
    Rx(2, 1) = sin(thetaX);
    Rx(2, 2) = cos(thetaX);

    Eigen::Matrix4f Ry = Eigen::Matrix4f::Identity();
    Ry(0, 0) = cos(thetaY);
    Ry(0, 2) = sin(thetaY);
    Ry(2, 0) = -sin(thetaY);
    Ry(2, 2) = cos(thetaY);

    Eigen::Matrix4f Rz = Eigen::Matrix4f::Identity();
    Rz(0, 0) = cos(thetaZ);
    Rz(0, 1) = -sin(thetaZ);
    Rz(1, 0) = sin(thetaZ);
    Rz(1, 1) = cos(thetaZ);

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T(0, 3) = 0.3;
    T(1, 3) = -0.2;
    T(2, 3) = 0.1;

    Eigen::Matrix4f theoretical_transform = Rz * Ry * Rx * T;

    // Log theoretical transformation
    cout << "Theoretical Transformation Matrix:" << endl;
    cout << theoretical_transform << endl;

    logFile << "\nTheoretical Transformation Matrix:" << endl;
    logFile << theoretical_transform << endl;
    logFile.flush();

    // Load point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe/data/armadillo_downsampled.pcd", *target) == -1)
    {
        PCL_ERROR("Failed to load target point cloud\n");
        logFile << "ERROR: Failed to load target point cloud" << endl;
        logFile.close();
        return -1;
    }
    cout << "Loaded " << target->size() << " points from the target point cloud" << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ivan/Desktop/Algoritmi strojnog vida/Kol_2/02_Point cloud registration/Vjezbe/data/armadillo_downsampled_gauss_20.pcd", *source) == -1)
    {
        PCL_ERROR("Failed to load source point cloud\n");
        logFile << "ERROR: Failed to load source point cloud" << endl;
        logFile.close();
        return -1;
    }
    cout << "Loaded " << source->size() << " points from the source point cloud" << endl;

    // Log initial parameters
    logFile << "\n=== Parameters ===" << endl;
    logFile << "Target points: " << target->size() << endl;
    logFile << "Source points: " << source->size() << endl;
    logFile << "Approximative overlap: " << approxOverlap << endl;
    logFile << "Lambda: " << lambda << endl;
    logFile << "Delta: " << delta << endl;
    logFile << "Number of threads: " << numThreads << endl;
    logFile << "Number of samples: " << numSamples << endl;
    logFile.flush();

    // Vectors to store results
    std::vector<double> times, rmses, norms, normalized_norms;

    // Main iteration loop
    const int iterations = 10;
    logFile << "\n=== Registration Results ===" << endl;
    logFile.flush();
    
    for (int iter = 0; iter < iterations; ++iter)
    {
        cout << "\n--- Starting iteration " << (iter + 1) << " ---" << endl;
        logFile << "\n--- Iteration " << (iter + 1) << " ---" << endl;
        
        time.tic();

        // Perform registration
        pcl::registration::KFPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> kfpcs;
        kfpcs.setInputSource(source);
        kfpcs.setInputTarget(target);
        kfpcs.setApproxOverlap(approxOverlap);
        kfpcs.setLambda(lambda);
        kfpcs.setDelta(delta, false);
        kfpcs.setNumberOfThreads(numThreads);
        kfpcs.setNumberOfSamples(numSamples);

        pcl::PointCloud<pcl::PointXYZ>::Ptr kpcs(new pcl::PointCloud<pcl::PointXYZ>);
        kfpcs.align(*kpcs);

        double elapsedTime = time.toc();
        times.push_back(elapsedTime);

        // Get transformation matrix
        Eigen::Matrix4f finalTransform = kfpcs.getFinalTransformation();
        
        logFile << "Final Transformation Matrix:" << endl;
        logFile << finalTransform << endl;

        // Calculate metrics
        double frobeniusNorm = calculateFrobeniusNorm(finalTransform - theoretical_transform);
        norms.push_back(frobeniusNorm);

        double groundTruthNorm = calculateFrobeniusNorm(theoretical_transform);
        double normalizedFrobeniusNorm = frobeniusNorm / groundTruthNorm;
        normalized_norms.push_back(normalizedFrobeniusNorm);

        // Calculate RMSE
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*source, *transformed_source, finalTransform);

        double rmse = 0.0;
        size_t min_size = std::min(transformed_source->size(), target->size());
        
        for (size_t i = 0; i < min_size; ++i)
        {
            Eigen::Vector3f p_transformed(
                transformed_source->points[i].x,
                transformed_source->points[i].y,
                transformed_source->points[i].z);

            Eigen::Vector3f p_target(
                target->points[i].x,
                target->points[i].y,
                target->points[i].z);

            rmse += (p_transformed - p_target).squaredNorm();
        }

        rmse = std::sqrt(rmse / min_size);
        rmses.push_back(rmse);

        // Log results for this iteration
        logFile << "Registration time: " << elapsedTime << " ms" << endl;
        logFile << "Frobenius Norm: " << frobeniusNorm << endl;
        logFile << "Normalized Frobenius Norm: " << normalizedFrobeniusNorm << endl;
        logFile << "RMSE: " << rmse << endl;
        logFile.flush(); // Immediate write after each iteration

        // Console output
        cout << "Registration time: " << elapsedTime << " ms" << endl;
        cout << "Frobenius Norm: " << frobeniusNorm << endl;
        cout << "Normalized Frobenius Norm: " << normalizedFrobeniusNorm << endl;
        cout << "RMSE: " << rmse << endl;
    }

    // Calculate statistics
    auto calculateMeanAndConfidence = [](const std::vector<double>& data) {
        double mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
        double variance = std::accumulate(data.begin(), data.end(), 0.0, 
                          [mean](double acc, double val) { return acc + (val - mean) * (val - mean); }) / data.size();
        double stddev = std::sqrt(variance);
        return std::make_tuple(mean, mean - 2 * stddev, mean + 2 * stddev);
    };

    auto [mean_time, ci_lower_time, ci_upper_time] = calculateMeanAndConfidence(times);
    auto [mean_rmse, ci_lower_rmse, ci_upper_rmse] = calculateMeanAndConfidence(rmses);
    auto [mean_norm, ci_lower_norm, ci_upper_norm] = calculateMeanAndConfidence(norms);
    auto [mean_normalized_norm, ci_lower_normalized_norm, ci_upper_normalized_norm] = calculateMeanAndConfidence(normalized_norms);

    // Final summary
    logFile << "\n=== SUMMARY ===" << endl;
    logFile << "Mean time: " << mean_time << " ms, CI: [" << ci_lower_time << ", " << ci_upper_time << "]" << endl;
    logFile << "Mean RMSE: " << mean_rmse << ", CI: [" << ci_lower_rmse << ", " << ci_upper_rmse << "]" << endl;
    logFile << "Mean Frobenius norm: " << mean_norm << ", CI: [" << ci_lower_norm << ", " << ci_upper_norm << "]" << endl;
    logFile << "Mean Normalized Frobenius norm: " << mean_normalized_norm << ", CI: [" << ci_lower_normalized_norm << ", " << ci_upper_normalized_norm << "]" << endl;
    
    cout << "\n=== SUMMARY ===" << endl;
    cout << "Mean time: " << mean_time << " ms, CI: [" << ci_lower_time << ", " << ci_upper_time << "]" << endl;
    cout << "Mean RMSE: " << mean_rmse << ", CI: [" << ci_lower_rmse << ", " << ci_upper_rmse << "]" << endl;
    cout << "Mean Frobenius norm: " << mean_norm << ", CI: [" << ci_lower_norm << ", " << ci_upper_norm << "]" << endl;
    cout << "Mean Normalized Frobenius norm: " << mean_normalized_norm << ", CI: [" << ci_lower_normalized_norm << ", " << ci_upper_normalized_norm << "]" << endl;

    // Final flush and close
    logFile.flush();
    logFile.close();
    
    cout << "\nLog file successfully written to: " << log_filename << endl;
    
    return 0;
}
