#ifndef LINE_MATCHING_HEADER_
#define LINE_MATCHING_HEADER_

#include "vector"
#include "Eigen/Dense"
#include "geometry_msgs/Pose.h"
#include "tuple"
#include <unistd.h>

#include "GlobalFeatureMap.hpp"
#include "LineClustering.hpp"

namespace CornerDetector {
    class LineMatching {
        private:
            class LineMatchingInfo {
                private:
                    Eigen::Vector3f g_start_line_;
                    Eigen::Vector3f g_end_line_;

                    Eigen::Vector3f o_start_line_;
                    Eigen::Vector3f o_end_line_;

                    float distance_;
                    float cosine_;
                public:
                    LineMatchingInfo(Eigen::Vector3f g_start_line, Eigen::Vector3f g_end_line,
                                     Eigen::Vector3f o_start_line, Eigen::Vector3f o_end_line,
                                     float distance, float cosine) :
                                     g_start_line_(g_start_line), g_end_line_(g_end_line), 
                                     o_start_line_(o_start_line), o_end_line_(o_end_line),
                                     distance_(distance), cosine_(cosine) {

                    }
                    ~LineMatchingInfo() {}

                    // Getter
                    Eigen::Vector3f g_start_line() const { return g_start_line_; }
                    Eigen::Vector3f g_end_line() const { return g_end_line_; }
                    Eigen::Vector3f o_start_line() const { return o_start_line_; }
                    Eigen::Vector3f o_end_line() const { return o_end_line_; }
                    float distance() const { return distance_; }
                    float cosine() const { return cosine_; }

                    // Setter
                    void g_start_line(Eigen::Vector3f g_start_line) { g_start_line_ = std::move(g_start_line); }
                    void g_end_line(Eigen::Vector3f g_end_line) { g_end_line_ = std::move(g_end_line); }
                    void o_start_line(Eigen::Vector3f o_start_line) { o_start_line_ = std::move(o_start_line); }
                    void o_end_line(Eigen::Vector3f o_end_line) { o_end_line_ = std::move(o_end_line); }
                    void distance(float distance) { distance_ = std::move(distance); }
                    void cosine(float cosine) { cosine_ = std::move(cosine); }
            };

            std::vector<LineMatchingInfo> VLMI_;
            std::tuple<bool, int> IsExist(Eigen::Vector3f start_of_line, Eigen::Vector3f end_of_line);
        public:
            LineMatching();
            ~LineMatching();

            void Matching(std::vector<GlobalLine> global_feature_lines,
                          std::vector<std::vector<float>> observed_lines,
                          geometry_msgs::Pose robot_point,
                          float &robot_velodyne_tf_x,
                          float &robot_velodyne_tf_y);
            float CosineSimilarity(Eigen::Vector3f u, Eigen::Vector3f v);
            std::vector<LineMatchingInfo> VLMI() const { return VLMI_; }
    }; 
}

#endif