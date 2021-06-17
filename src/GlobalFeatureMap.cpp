#include "GlobalFeatureMap.hpp"

namespace CornerDetector {
    GlobalFeatureMap::GlobalFeatureMap() {

    }

    GlobalFeatureMap::~GlobalFeatureMap() {

    }

    void GlobalFeatureMap::Run() {
        SetParameter();
        // ROS_INFO("File Path Points: %s", file_path_points_);
        // ROS_INFO("File Path Lines: %s", file_path_lines_);


        // thread point and line
        std::thread t1(&GlobalFeatureMap::PreprocessingPoints, this);
        std::thread t2(&GlobalFeatureMap::PreprocessingLines, this);
        if(t1.joinable() == true)
            t1.join();
        if(t2.joinable() == true)
            t2.join();
    }

    void GlobalFeatureMap::SetParameter() {
        n.getParam("/file_path_points", file_path_points_);
        n.getParam("/file_path_lines", file_path_lines_);
    } 

    void GlobalFeatureMap::PreprocessingPoints() {
        std::ifstream point_file(file_path_points_);
        if(point_file.is_open()) {
            std::string line, parsed;
            while(std::getline(point_file, line)) {
                // std::cout << "global corner data: " << line << std::endl;
                std::vector<std::string> results;
                boost::split(results, line, [](char c){return c == ',';});

                // int id = std::stoi(results[0]);
                Eigen::Vector3f v(std::stof(results[1]), std::stof(results[2]), std::stof(results[3]));
                GlobalPoint p(v);
                feature_points_.push_back(p);
            }
            point_file.close();
        }
    }

    void GlobalFeatureMap::PreprocessingLines() {
        std::ifstream line_file(file_path_lines_);
        if(line_file.is_open()) {
            // <start_point<x, y, z>, end_point<x, y, z>>, <s, e> <s, e> ...
            std::string line, parsed;
            while(std::getline(line_file, line)) {
                // std::cout << "global line data: " << line << std::endl;
                std::vector<std::string> results;
                boost::split(results, line, [](char c){return c == ',';});
                Eigen::Vector3f start_point(std::stof(results[1]), std::stof(results[2]), std::stof(results[3]));

                std::getline(line_file, line);
                // std::cout << "global line data: " << line << std::endl;
                boost::split(results, line, [](char c){return c == ',';});
                Eigen::Vector3f end_point(std::stof(results[1]), std::stof(results[2]), std::stof(results[3]));

                GlobalLine l(start_point, end_point);
                feature_lines_.push_back(l);
            }
            line_file.close();
        } else {
            ROS_ERROR("Line data file not open");
        }
    }
}
