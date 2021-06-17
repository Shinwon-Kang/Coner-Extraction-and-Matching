#ifndef GLOBAL_FEATURE_MAP_HEADER_
#define GLOBAL_FEATURE_MAP_HEADER_

#include "ros/ros.h"
#include "Eigen/Dense"
#include "vector"
#include "fstream"
#include "iostream"
#include "string"
#include "boost/algorithm/string.hpp"
#include "thread"
#include "iostream"

namespace CornerDetector {

class GlobalPoint {
    private:
        Eigen::Vector3f point_;

    public:
        GlobalPoint(Eigen::Vector3f point) {
            point_ = point;
        }
        ~GlobalPoint() {}

        // Getter
        Eigen::Vector3f point() const { return point_; }
};

class GlobalLine {
    private:
        Eigen::Vector3f start_point_;
        Eigen::Vector3f end_point_;
    public:
        GlobalLine(Eigen::Vector3f start_point, Eigen::Vector3f end_point) {
            start_point_ = start_point;
            end_point_ = end_point;
        }
        ~GlobalLine() {}

        // Getter
        Eigen::Vector3f start_point() const { return start_point_; }
        Eigen::Vector3f end_point() const { return end_point_; }
};

class GlobalFeatureMap {
    private:
        ros::NodeHandle n;
        
        std::string file_path_points_;
        std::string file_path_lines_;

        std::vector<GlobalPoint> feature_points_;
        std::vector<GlobalLine> feature_lines_;
    public:
        GlobalFeatureMap();
        ~GlobalFeatureMap();

        void SetParameter();
        void Run();
        void PreprocessingPoints();
        void PreprocessingLines();
        
        // std::vector<GlobalPoint> getFeaturePoints();
        std::vector<GlobalPoint> feature_points() const { return feature_points_; }
        std::vector<GlobalLine> feature_lines() const { return feature_lines_; }
    };
}

#endif