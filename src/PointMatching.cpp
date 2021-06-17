#include "PointMatching.hpp"

namespace CornerDetector
{
    PointMatching::PointMatching() {

    }

    PointMatching::~PointMatching() {

    }

    // GlobalPoint: GP, ObservationPoint: OP
    void PointMatching::Matching(Eigen::MatrixXf global_point, 
                                 std::vector<AccumulateFeatures::Features> observation_point, 
                                 geometry_msgs::Pose robot_point,
                                 float &robot_velodyne_tf_x,
                                 float &robot_velodyne_tf_y) {

        // for(int i = 0; i < observation_point.size(); i++) {
        //     float x = (observation_point[i][0] + robot_velodyne_tf_x) * cosf(robot_point.orientation.z) 
        //                 - (observation_point[i][1] + robot_velodyne_tf_y) * sinf(robot_point.orientation.z);
        //     float y = (observation_point[i][0] + robot_velodyne_tf_x) * sinf(robot_point.orientation.z) 
        //                 + (observation_point[i][1] + robot_velodyne_tf_y) * cosf(robot_point.orientation.z);

        //     Eigen::Vector3f v = {robot_point.position.x + x,
        //                          robot_point.position.y + y,
        //                          0};
        for(int i = 0; i < observation_point.size(); i++) {
            Eigen::Vector3f v = {observation_point[i].f[0],
                                 observation_point[i].f[1],
                                 0};

            Eigen::MatrixXf::Index min_index;
            float min_distance = (global_point.colwise() - v).colwise().squaredNorm().minCoeff(&min_index);
            // Eigen::MatrixXf distance_matrix = (global_point.colwise() - v).colwise().squaredNorm();
            // distance_matrix.minCoeff(&min_index);


            float gp_x = global_point(0, min_index);
            float gp_y = global_point(1, min_index);
            float gp_z = global_point(2, min_index);
            
            float t_op_x = observation_point[i].f[0] - robot_point.position.x;
            float t_op_y = observation_point[i].f[1] - robot_point.position.y;

            float op_x = (t_op_x + robot_velodyne_tf_x) * cosf(robot_point.orientation.z) 
                        + (t_op_y + robot_velodyne_tf_y) * sinf(robot_point.orientation.z);

            float op_y = - (t_op_x + robot_velodyne_tf_x) * sinf(robot_point.orientation.z) 
                        + (t_op_y + robot_velodyne_tf_y) * cosf(robot_point.orientation.z);

            // float op_x = observation_point[i][0];
            // float op_y = observation_point[i][1];
            float op_z = 0.0;
            
            // float min_distance = distance_matrix(min_index);

            // threshold distance && duplicate Other Point
            bool is_exist;
            int index_exist;
            std::tie(is_exist, index_exist) = IsExist(gp_x, gp_y, gp_z);
            if(is_exist) {
                // 중복 (중복된 포인트와 거리 비교)
                if(VPMI[index_exist].distance() > min_distance) {
                    // 기존 데이터랑 변경 update
                    Eigen::Vector3f op(op_x, op_y, op_z);

                    VPMI[index_exist].op(op);
                    VPMI[index_exist].distance(min_distance);
                }
            } else {
                if(min_distance < 0.3) {
                    // not 중복 (추가)
                    Eigen::Vector3f gp(gp_x, gp_y, gp_z);
                    Eigen::Vector3f op(op_x, op_y, op_z);

                    PointMatchingInfo pi(gp, op, min_distance);
                    VPMI.push_back(pi);
                }
            }
        }
    }

    std::tuple<bool, int> PointMatching::IsExist(float gp_x, float gp_y, float gp_z) {
        if(VPMI.empty()) { 
            return std::make_tuple(false, -1);
        } else {
            for(int i = 0; i < VPMI.size(); i++) {
                if(VPMI[i].gp()(0) == gp_x && VPMI[i].gp()(1) == gp_y && VPMI[i].gp()(2) == gp_z) {
                    return std::make_tuple(true, i);
                }
            }
        }
        return std::make_tuple(false, -1);
    }
}
