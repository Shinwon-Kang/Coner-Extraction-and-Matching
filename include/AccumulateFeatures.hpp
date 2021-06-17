#ifndef ACCUMULATE_FEATURES_HEADER_
#define ACCUMULATE_FEATURES_HEADER_

#include "Eigen/Dense"
#include "vector"
#include "cmath"
#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "chrono"

#include "VectorUtils.hpp"

namespace CornerDetector
{
class AccumulateFeatures {
    private:
    public:
        class Features {
            public:
                std::vector<float> f;
                std::chrono::system_clock::time_point time;
                Features(std::vector<float> f, std::chrono::system_clock::time_point time) {
                    this->f = f;
                    this->time = time;
                }
        };
        std::vector<Features> old_f;

        void Run(std::vector<std::vector<float>> new_f,
                 geometry_msgs::Pose robot_point,
                 float &robot_velodyne_tf_x,
                 float &robot_velodyne_tf_y);
};
}

#endif
