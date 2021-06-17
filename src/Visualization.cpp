#include "Visualization.hpp"

namespace CornerDetector
{
    Visualization::Visualization() {

    }

    Visualization::~Visualization() {

    }

    void Visualization::Run(const LineClustering &line_clustering,
                            visualization_msgs::Marker &epcc,
                            visualization_msgs::Marker &down,
                            visualization_msgs::Marker &corner,
                            visualization_msgs::Marker &tmp) {

        // EPCC Line Clustering
        EPCCMarker_v2(line_clustering.cluster_lines, epcc);

        // DownSampling Point
        // DownsamplingMarker(line_clustering.cluster_points, down);

        // Corner Point
        EPCCCornerMarker(line_clustering.corner_points, corner);
    }

    void Visualization::vis(std::vector<AccumulateFeatures::Features> old_f, visualization_msgs::Marker &corner) {
        corner.header.frame_id = "/map";
        corner.header.stamp = ros::Time::now();
        corner.ns = "breakpoint_detector";
        corner.action = visualization_msgs::Marker::ADD;
        corner.pose.orientation.w = 1.0;

        corner.id = 879;
        corner.type = visualization_msgs::Marker::POINTS;

        corner.scale.x = 0.2;
	    corner.scale.y = 0.2;

        corner.color.r = 0.0;
        corner.color.g = 0.0;
        corner.color.b = 1.0;
        corner.color.a = 1.0;

        for(int i = 0; i < old_f.size(); i++) {
            geometry_msgs::Point p;
            p.x = old_f[i].f[0];
            p.y = old_f[i].f[1];
            p.z = 0.0;

            corner.points.push_back(p);
        }
    }

    void Visualization::EPCCMarker(LineClustering lc) {
        visualization_msgs::Marker epcc;
        epcc.header.frame_id = "/base_link";
        epcc.header.stamp = ros::Time::now();
        epcc.ns = "breakpoint_detector";
        epcc.action = visualization_msgs::Marker::ADD;
        epcc.pose.orientation.w = 1.0;

        epcc.id = 9;
        epcc.type = visualization_msgs::Marker::LINE_LIST;

        epcc.scale.x = 0.01;
        epcc.color.r = 0.0;
        epcc.color.g = 1.0;
        epcc.color.b = 0.0;
        epcc.color.a = 1.0;
        
        for(int i = 0; i < lc.cluster_lines.size(); i++) {
            if(lc.cluster_lines[i].points_in_line.size() >= 2) {
                for(int j = 0; j < lc.cluster_lines[i].points_in_line.size() - 1; j++) {
                    Eigen::Vector2f line_1 = lc.cluster_lines[i].points_in_line[j];
                    Eigen::Vector2f line_2 = lc.cluster_lines[i].points_in_line[j + 1];
                    
                    geometry_msgs::Point gp1, gp2;
                    gp1.x = line_1(0, 0);
                    gp1.y = line_1(1, 0);
                    gp1.z = 0;

                    gp2.x = line_2(0, 0);
                    gp2.y = line_2(1, 0);
                    gp2.z = 0;
                    
                    epcc.points.push_back(gp1);
                    epcc.points.push_back(gp2);
                }
            }
        }
    }

    void Visualization::EPCCMarker_v2(std::vector<LineClustering::Line> cluster_lines, visualization_msgs::Marker &epcc) {
        epcc.header.frame_id = "/base_link";
        epcc.header.stamp = ros::Time::now();
        epcc.ns = "breakpoint_detector";
        epcc.action = visualization_msgs::Marker::ADD;
        epcc.pose.orientation.w = 1.0;

        epcc.id = 9;
        epcc.type = visualization_msgs::Marker::LINE_LIST;

        epcc.scale.x = 0.05;
        epcc.color.r = 0.0;
        epcc.color.g = 1.0;
        epcc.color.b = 0.0;
        epcc.color.a = 1.0;

        std_msgs::ColorRGBA c;
        c.r = 1.0;
        c.g = 0.0;
        c.b = 0.0;
        c.a = 1.0;

        for(int i = 0; i < cluster_lines.size(); i++) {
            if(cluster_lines[i].points_in_line.size() >= 2) {
                for(int j = 0; j < cluster_lines[i].points_in_line.size() - 1; j++) {
                    Eigen::Vector2f line_1 = cluster_lines[i].points_in_line[j];
                    Eigen::Vector2f line_2 = cluster_lines[i].points_in_line[j + 1];
                    
                    geometry_msgs::Point gp1, gp2;
                    gp1.x = line_1(0, 0);
                    gp1.y = line_1(1, 0);
                    gp1.z = 0;

                    gp2.x = line_2(0, 0);
                    gp2.y = line_2(1, 0);
                    gp2.z = 0;

                    epcc.colors.push_back(c);
                    epcc.points.push_back(gp1);
                    epcc.colors.push_back(c);
                    epcc.points.push_back(gp2);
                }
            }   
        }
        c.g += 0.2;
    }

    void Visualization::DownsamplingMarker(std::vector<Eigen::Vector2f> cluster_points, visualization_msgs::Marker &down) {
        down.header.frame_id = "/base_link";
        down.header.stamp = ros::Time::now();
        down.ns = "breakpoint_detector";
        down.action = visualization_msgs::Marker::ADD;
        down.pose.orientation.w = 1.0;

        down.id = 13;
        down.type = visualization_msgs::Marker::POINTS;

        down.scale.x = 0.05;
        down.color.r = 1.0;
        down.color.g = 0.0;
        down.color.b = 1.0;
        down.color.a = 1.0;

        std_msgs::ColorRGBA c;
        c.r = 0.2;
        c.g = 0.7;
        c.b = 1.0;
        c.a = 1.0;

        BOOST_FOREACH(const Eigen::Vector2f line, cluster_points) {
            geometry_msgs::Point gp1;
            gp1.x = line(0, 0);
            gp1.y = line(1, 0);
            gp1.z = 0;

            down.colors.push_back(c);            
            down.points.push_back(gp1);
        }
    }

    void Visualization::EPCCCornerMarker(std::vector<std::vector<float>> corner_points, visualization_msgs::Marker &corner) {
        corner.header.frame_id = "/base_lnk";
        corner.header.stamp = ros::Time::now();
        corner.ns = "breakpoint_detector";
        corner.action = visualization_msgs::Marker::ADD;
        corner.pose.orientation.w = 1.0;

        corner.id = 15;
        corner.type = visualization_msgs::Marker::POINTS;

        corner.scale.x = 0.2;
	    corner.scale.y = 0.2;

        corner.color.r = 1.0;
        corner.color.g = 0.0;
        corner.color.b = 1.0;
        corner.color.a = 1.0;

        BOOST_FOREACH(const std::vector<float> cord, corner_points) {
            geometry_msgs::Point gp1;
            // gp1.x = cord[0] - 0.7;
            gp1.x = cord[0];
            gp1.y = cord[1];
            gp1.z = 0;

            corner.points.push_back(gp1);
        }   
    }

    void Visualization::TempMarker(std::vector<std::vector<float>> temp, visualization_msgs::Marker &tmp) {
        tmp.header.frame_id = "/base_link";
        tmp.header.stamp = ros::Time::now();
        tmp.ns = "breakpoint_detector";
        tmp.action = visualization_msgs::Marker::ADD;
        tmp.pose.orientation.w = 1.0;

        tmp.id = 23;
        tmp.type = visualization_msgs::Marker::LINE_LIST;

        tmp.scale.x = 0.1;
        
        tmp.color.r = 0.0;
        tmp.color.g = 1.0;
        tmp.color.b = 0.0;
        tmp.color.a = 1.0;

        std_msgs::ColorRGBA c;
        c.r = 1.0;
        c.g = 0.0;
        c.b = 0.0;
        c.a = 1.0;
        
        geometry_msgs::Point gp1, gp2;
        if(temp.size() > 0) {
            for(int j = 0; j < temp.size(); j+=2) {
                std::vector<float> cord1 = temp[j];
                std::vector<float> cord2 = temp[j + 1];
            
                gp1.x = cord1[0];
                gp1.y = cord1[1];
                gp1.z = 0;

                gp2.x = cord2[0];
                gp2.y = cord2[1];
                gp2.z = 0;

                tmp.colors.push_back(c);
                tmp.points.push_back(gp1);
                tmp.colors.push_back(c);
                tmp.points.push_back(gp2);
            }
        }
    }
}
