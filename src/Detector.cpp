#include "ros/ros.h"
#include "Eigen/Dense"

#include "RobotState.hpp"
#include "GlobalFeatureMap.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "Detector");

    CornerDetector::RobotState robot;
/*
    Eigen::MatrixXf m(3, 4);
    Eigen::VectorXf v(3);

    m << 1, 2, 1.5, 6,
         1, 2, 0.5, 7,
         1, 0, 0, 0;

    v << 0,
         0,
         0;

    Eigen::MatrixXf::Index index;

    // ros::Time start = ros::Time::now();

    (m.colwise() - v).colwise().squaredNorm().minCoeff(&index);

    // ros::Time end = ros::Time::now();
    // ros::Duration term = end - start;
    // double time = (double)term.nsec/1000000000;

    std::cout << "Nearest neighbour is column " << index << ":" << std::endl;
    std::cout << m.col(index) << std::endl;
    std::cout << (m.colwise() - v).colwise().squaredNorm() << std::endl;
    // std::cout << "time: " << time << std::endl;
*/
    return 0;
}