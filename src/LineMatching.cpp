#include "LineMatching.hpp"

namespace CornerDetector
{
    LineMatching::LineMatching() {

    }

    LineMatching::~LineMatching() {

    }

    void LineMatching::Matching(std::vector<GlobalLine> global_feature_lines,
                                std::vector<std::vector<float>> observed_lines,
                                geometry_msgs::Pose robot_point,
                                float &robot_velodyne_tf_x,
                                float &robot_velodyne_tf_y) {  
        // TODO : Distance
        for(int i = 0; i < global_feature_lines.size(); i++) {
            int min_distance, index;
            Eigen::Vector3f g_center_point((global_feature_lines[i].end_point().x() + global_feature_lines[i].start_point().x()) / 2,
                                           (global_feature_lines[i].end_point().y() + global_feature_lines[i].start_point().y()) / 2,
                                           (global_feature_lines[i].end_point().z() + global_feature_lines[i].start_point().z()) / 2);

            Eigen::Vector3f u(global_feature_lines[i].start_point().x() - global_feature_lines[i].end_point().x(),
                                global_feature_lines[i].start_point().y() - global_feature_lines[i].end_point().y(),
                                global_feature_lines[i].start_point().z() - global_feature_lines[i].end_point().z());

            for(int j = 0; j < observed_lines.size(); j+=2) {
                // transfrom robot coordinate
                // Eigen::Vector3f o_center_point((observed_lines[j][0] + observed_lines[j + 1][0]) / 2,
                //                                (observed_lines[j][1] + observed_lines[j + 1][0]) / 2,
                //                                (observed_lines[j][2] + observed_lines[j + 1][0]) / 2);
                float observed_lines_start_x = (observed_lines[j][0] + robot_velodyne_tf_x) * cosf(robot_point.orientation.z) 
                                               - (observed_lines[j][1] + robot_velodyne_tf_y) * sinf(robot_point.orientation.z);
                float observed_lines_start_y = (observed_lines[j][0] + robot_velodyne_tf_x) * sinf(robot_point.orientation.z) 
                                               + (observed_lines[j][1] + robot_velodyne_tf_y) * cosf(robot_point.orientation.z);
                float observed_lines_start_z = 0;
                Eigen::Vector3f o_start_point(robot_point.position.x + observed_lines_start_x,
                                              robot_point.position.y + observed_lines_start_y,
                                              0);

                float observed_lines_end_x = (observed_lines[j + 1][0] + robot_velodyne_tf_x) * cosf(robot_point.orientation.z) 
                                               - (observed_lines[j + 1][1] + robot_velodyne_tf_y) * sinf(robot_point.orientation.z);
                float observed_lines_end_y = (observed_lines[j + 1][0] + robot_velodyne_tf_x) * sinf(robot_point.orientation.z) 
                                               + (observed_lines[j + 1][1] + robot_velodyne_tf_y) * cosf(robot_point.orientation.z);
                float observed_lines_end_z = 0;
                Eigen::Vector3f o_end_point(robot_point.position.x + observed_lines_end_x,
                                            robot_point.position.y + observed_lines_end_y,
                                            0);

                Eigen::Vector3f v(o_end_point.x() - o_start_point.x(),
                                  o_end_point.y() - o_start_point.y(),
                                  o_end_point.z() - o_start_point.z());
    
                float cosine_similarity = CosineSimilarity(u, v);
                // parallel
                if(cosine_similarity > 0.9 || cosine_similarity < -0.9) {
                    // DISTANCE
                    Eigen::Vector3f o_center_point((o_start_point.x() + o_end_point.x()) / 2,
                                                   (o_start_point.y() + o_end_point.y()) / 2,
                                                   (o_start_point.z() + o_end_point.z()) / 2);
                    
                    Eigen::MatrixXf::Index min_index;
                    float distance = (g_center_point.colwise() - o_center_point).colwise().squaredNorm().minCoeff(&min_index);
                    if(distance < 0.1) {
                        // ROS_ERROR("DISTANCE IS samller than 0.1");

                        bool is_exist;
                        int index_exist;
                        std::tie(is_exist, index_exist) = IsExist(o_start_point, o_end_point);
                        if(is_exist) {
                            if(VLMI_[index_exist].distance() > distance) {
                                VLMI_[index_exist].o_start_line(o_start_point);
                                VLMI_[index_exist].o_end_line(o_end_point);
                                VLMI_[index_exist].distance(distance);
                                VLMI_[index_exist].cosine(cosine_similarity);
                            }
                        } else {
                            LineMatchingInfo li(global_feature_lines[i].start_point(),
                                                global_feature_lines[i].end_point(),
                                                o_start_point,
                                                o_end_point,
                                                distance,
                                                cosine_similarity);
                            VLMI_.push_back(li);  
                        }
                    } else {
                    }
                }
                // vertical 
                else {
                }
            }
        }
        // TODO : CosineSimilarity
    }

    std::tuple<bool, int> LineMatching::IsExist(Eigen::Vector3f start_of_line, Eigen::Vector3f end_of_line) {
        if(VLMI_.empty()) {
            return std::make_tuple(false, -1);
        } else {
            for(int i = 0; i < VLMI_.size(); i++) {
                // start_point, end_point 
                if((VLMI_[i].o_start_line().x() == start_of_line.x() && VLMI_[i].o_start_line().y() == start_of_line.y() && VLMI_[i].o_start_line().z() == start_of_line.z())
                    && (VLMI_[i].o_end_line().x() == end_of_line.x() && VLMI_[i].o_end_line().y() == end_of_line.y() && VLMI_[i].o_end_line().z() == end_of_line.z())) {
                        return std::make_tuple(true, i);
                    }
            }
        }
        return std::make_tuple(false, -1);
    }

    float LineMatching::CosineSimilarity(Eigen::Vector3f u, Eigen::Vector3f v) {
        float x = u(0) * v(0) + u(1) * v(1) + u(2) * v(2);
        float y = u.norm() * v.norm();
        float cosine_similarity = x / y;

        return cosine_similarity;
    }

}
