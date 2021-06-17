#ifndef VISUALIZATION_HEADER_
#define VISUALIZATION_HEADER_

#include "vector"
#include "Eigen/Dense"
#include "LineClustering.hpp"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "boost/foreach.hpp"
#include "AccumulateFeatures.hpp"

namespace CornerDetector
{
class Visualization {
    private:
        void EPCCMarker(LineClustering lc);
        void EPCCMarker_v2(std::vector<LineClustering::Line> cluster_lines, visualization_msgs::Marker &epcc);
        void DownsamplingMarker(std::vector<Eigen::Vector2f> cluster_points, visualization_msgs::Marker &down);
        void EPCCCornerMarker(std::vector<std::vector<float>> corner_points, visualization_msgs::Marker &corner);
        void TempMarker(std::vector<std::vector<float>> temp, visualization_msgs::Marker &tmp);
    public:
        Visualization();
        ~Visualization();

        void Run(const LineClustering &line_clustering,
                visualization_msgs::Marker &epcc,
                visualization_msgs::Marker &down,
                visualization_msgs::Marker &corner,
                visualization_msgs::Marker &tmp);

        void vis(std::vector<AccumulateFeatures::Features> old_f, visualization_msgs::Marker &corner);
};
} // namespace CornerDetector

#endif