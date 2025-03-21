// test_eigen.cpp
#include <iostream>
#include <Eigen/Dense>  // Explicitly include Eigen
// #include <Vector3Df.h>  // Your flair Vector3Df

// using namespace flair::core;

int main() {
    // Create an Eigen vector and perform operations
    Eigen::Vector3d eigen_vec(1.0, 2.0, 3.0);
    
    // Some Eigen operations
    Eigen::Vector3d doubled = eigen_vec * 2.0;
    double norm = eigen_vec.norm();
    
    std::cout << "Eigen vector: " << eigen_vec.transpose() << std::endl;
    std::cout << "Doubled: " << doubled.transpose() << std::endl;
    std::cout << "Norm: " << norm << std::endl;
    
    // // Convert between Eigen and Vector3Df
    // Vector3Df flair_vec(1.0f, 2.0f, 3.0f);
    // Eigen::Vector3d from_flair(flair_vec.x, flair_vec.y, flair_vec.z);
    
    // std::cout << "Converted from Vector3Df to Eigen: " << from_flair.transpose() << std::endl;
    
    // std::cout << "Eigen include is working correctly!" << std::endl;
    return 0;
}