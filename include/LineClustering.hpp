#ifndef LINE_CLUSTERING_HEADER_
#define LINE_CLUSTERING_HEADER_

#include "Eigen/Dense"
#include "vector"
#include "cmath"
#include "ros/ros.h"

#include "VectorUtils.hpp"

namespace CornerDetector
{
class LineClustering {
    private:
    public:
        class Line {
            public:
                std::vector<Eigen::Vector2f> points_in_line;
                std::vector<Eigen::Vector2f> mean_value;
                Eigen::Matrix2f covariance_matrix;
                Eigen::Vector2f normal_vector;
                float cluster_distance_variance;
                float orthogonal_distance;

                Line(std::vector<Eigen::Vector2f> points_in_line) {
                    this->points_in_line = points_in_line;
                }
                ~Line() {
                    
                }
        };
        std::vector<Eigen::Vector2f> cluster_points;
        std::vector<Line> cluster_lines;

        std::vector<std::vector<float>> corner_points;
        //TODO: remove this
        std::vector<std::vector<float>> temp;

        int index_of_cluster;
        int k_min;

        // LineClustering(std::vector<Breakpoint> corner_detector, int breakpoint_index_1, int breakpoint_index_2, bool is_extra, int extra_waypoint_1, int extra_waypoint_2);
        LineClustering(std::vector<Eigen::Vector2f> cluster_points);
        ~LineClustering();

        void EPCC();
        void InitializeVariable(std::vector<Eigen::Vector2f> buffer);
        bool IsConsistent(std::vector<Eigen::Vector2f> buffer);
        Eigen::Vector2f MeanValue(int index, Eigen::Vector2f current_point);
        Eigen::Matrix2f CovarianceMatrix(int index, Eigen::Vector2f current_point);
        Eigen::Vector2f NormalVector();
        float OrthogonalDistance(Eigen::Vector2f point);
        float ClusterdistanceVariance(int index);
        void DetermineCorner();
        float GetTwoLineAngle(Line line1, Line line2);
};
} // namespace CornerDetector

#endif