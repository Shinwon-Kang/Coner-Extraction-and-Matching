#include "AccumulateFeatures.hpp"

namespace CornerDetector
{
    void AccumulateFeatures::Run(std::vector<std::vector<float>> new_f,
                                 geometry_msgs::Pose robot_point,
                                 float &robot_velodyne_tf_x,
                                 float &robot_velodyne_tf_y) {
        if(this->old_f.size() == 0) {
            for(int i = 0; i < new_f.size(); i++) {
                std::vector<float> p = new_f[i];

                // std::cout << "before: " << p[0] << " , " << p[1] << std::endl;
                float x = (p[0] + robot_velodyne_tf_x) * cosf(robot_point.orientation.z) 
                            - (p[1] + robot_velodyne_tf_y) * sinf(robot_point.orientation.z);
                float y = (p[0] + robot_velodyne_tf_x) * sinf(robot_point.orientation.z) 
                            + (p[1] + robot_velodyne_tf_y) * cosf(robot_point.orientation.z);
                x = robot_point.position.x + x;
                y = robot_point.position.y + y;

                std::vector<float> g_p{x, y};
                auto start = std::chrono::high_resolution_clock::now();
                this->old_f.push_back(Features(g_p, start));
            }

            return;
        }

        // std::cout << "feature size: " << this->old_f.size() << std::endl;
        while(new_f.size() > 0) {
            int isExist = -1;
            std::vector<float> p = new_f[0];
            new_f.erase(new_f.begin());

            float x = (p[0] + robot_velodyne_tf_x) * cosf(robot_point.orientation.z) 
                        - (p[1] + robot_velodyne_tf_y) * sinf(robot_point.orientation.z);
            float y = (p[0] + robot_velodyne_tf_x) * sinf(robot_point.orientation.z) 
                        + (p[1] + robot_velodyne_tf_y) * cosf(robot_point.orientation.z);
            x = robot_point.position.x + x;
            y = robot_point.position.y + y;            

            std::vector<float> g_p{x, y};
            for(int i = 0; i < this->old_f.size(); i++) {
                std::vector<float> q = this->old_f[i].f;
                float dis = VectorUtils::GetTwoPointsDistance(q, g_p);

                if(dis < 0.5) {
                    isExist = i;
                    
                    this->old_f[i].f[0] = g_p[0];
                    this->old_f[i].f[1] = g_p[1];
                }
            }
            if(isExist == -1) {
                auto start = std::chrono::high_resolution_clock::now();
                this->old_f.push_back(Features(g_p, start));
            }
        }

        // 일정 프레임 뒤에 없어져야됨
        for(int i = 0; i < this->old_f.size(); i++) {
            auto finish = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = finish - this->old_f[i].time;
            if(elapsed.count() > 3) {
                this->old_f.erase(this->old_f.begin() + i);
            }
        }
    }
}