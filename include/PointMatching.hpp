#ifndef POINT_MATCHING_HEADER_
#define POINT_MATCHING_HEADER_

#include "vector"
#include "Eigen/Dense"
#include "geometry_msgs/Pose.h"
#include "tuple"

#include "AccumulateFeatures.hpp"

namespace CornerDetector {
    class PointMatching {
        private:
            class PointMatchingInfo {
                private:
                    Eigen::Vector3f gp_;
                    Eigen::Vector3f op_;
                    float distance_;
                public:
                    PointMatchingInfo(Eigen::Vector3f gp, Eigen::Vector3f op, float distance) {
                        gp_ = gp;
                        op_ = op;
                        distance_ = distance;
                    }

                    ~PointMatchingInfo() {}

                    // Getter
                    Eigen::Vector3f gp() const { return gp_; }
                    Eigen::Vector3f op() const { return op_; }
                    float distance() const { return distance_; }

                    // Setter
                    void gp(Eigen::Vector3f gp) { gp_ = std::move(gp); }
                    void op(Eigen::Vector3f op) { op_ = std::move(op); }
                    void distance(float distance) { distance_ = distance; }
            };

            std::tuple<bool, int> IsExist(float gp_x, float gp_y, float gp_z);
        public:
            std::vector<PointMatchingInfo> VPMI;

            PointMatching();
            ~PointMatching();

            void Matching(Eigen::MatrixXf global_point, 
                          std::vector<AccumulateFeatures::Features> observation_point, 
                          geometry_msgs::Pose robot_point,
                          float &robot_velodyne_tf_x,
                          float &robot_velodyne_tf_y);
    }; 
}

#endif