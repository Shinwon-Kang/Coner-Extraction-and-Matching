#include "LineClustering.hpp"

namespace CornerDetector
{
    LineClustering::LineClustering(std::vector<Eigen::Vector2f> cluster_points) : cluster_points(cluster_points) {
    
    }

    LineClustering::~LineClustering() {

    }

    void LineClustering::EPCC() {
        // ROS_INFO("Enter EPCC method");
        // initialization
        k_min = 4;
        index_of_cluster = 0;
        std::vector<Eigen::Vector2f> buffer;

        // TODO: IsConsistent?
        for(int i = 0; i < cluster_points.size(); i++) {
            buffer.push_back(cluster_points[i]);

            if(buffer.size() == k_min) {
                if(IsConsistent(buffer)) {
                cluster_lines.push_back(Line (buffer));
                break;
                } else {
                    buffer.clear();
                    i -= (k_min - 1);
                }
            } 
        }
        if(buffer.size() < k_min) {
            return;
        }

        // init variance
        InitializeVariable(buffer);
        buffer.clear();

        float od;
        for(int i = k_min; i < cluster_points.size(); i++) {
            Line line = cluster_lines[index_of_cluster];

            Eigen::Vector2f current_point = cluster_points[i];
            od = OrthogonalDistance(current_point);

            // add condition distance
            std::vector<float> pre_point = {line.points_in_line[line.points_in_line.size() - 1](0, 0), line.points_in_line[line.points_in_line.size() - 1](1, 0)};
            std::vector<float> l_point = {current_point(0, 0), current_point(1, 0)};
            float dis = VectorUtils::GetTwoPointsDistance(pre_point, l_point);
            if((od == 0 || (od < k_min * sqrtf(line.cluster_distance_variance))) && (dis < 0.5)) {
                // add current_point
                cluster_lines[index_of_cluster].points_in_line.push_back(current_point);
                int index = cluster_lines[index_of_cluster].points_in_line.size() - 1;
                
                // update value...
                cluster_lines[index_of_cluster].mean_value.push_back(MeanValue(index, current_point));
                cluster_lines[index_of_cluster].covariance_matrix = CovarianceMatrix(index, current_point);
                cluster_lines[index_of_cluster].normal_vector = NormalVector();
                cluster_lines[index_of_cluster].orthogonal_distance = OrthogonalDistance(current_point);
                cluster_lines[index_of_cluster].cluster_distance_variance = ClusterdistanceVariance(index);

                buffer.clear();
            } else {
                buffer.push_back(current_point);
                if(buffer.size() >= k_min) {
                    if(IsConsistent(buffer)) {
                        // add new cluster
                        index_of_cluster++;
                        cluster_lines.push_back(Line (buffer));
                        
                        // update value...
                        InitializeVariable(buffer);

                        buffer.clear();
                    } else {
                        buffer.erase(buffer.begin());
                    }
                }
            }
        }
    }

    bool LineClustering::IsConsistent(std::vector<Eigen::Vector2f> buffer) {
        // Eigen::Vector2f mean = MeanValue(0, buffer);
        // 1. points to distnace < threshold
        // 2. consistnet

        // Point to Point distance
        float dis_threshold = 0.15;
        // Point to Line distance
        float sub_threshold = 0.05;
        std::vector<float> pre_point = {buffer[0](0, 0), buffer[0](1, 0)};
        std::vector<float> l_point = {buffer[buffer.size() - 1](0, 0), buffer[buffer.size() - 1](1, 0)};
        std::vector<float> equation = VectorUtils::GetEquationParameter(pre_point, l_point);

        float sub = 0;
        for(int i = 1; i < buffer.size() - 1; i++) {
            std::vector<float> p = {buffer[i](0, 0), buffer[i](1, 0)};
            float dis = VectorUtils::GetTwoPointsDistance(pre_point, p);
            if(dis > dis_threshold) {
                return false;
            }

            sub += VectorUtils::GetPointToEquationDistance(equation, p);
            pre_point = p;
        }

        // check distance of last point
        float dis = VectorUtils::GetTwoPointsDistance(pre_point, l_point);
        if(dis > dis_threshold) {
            return false;
        }

        if(sub < sub_threshold) {
            return true;
        }

        return false;
    }

    void LineClustering::InitializeVariable(std::vector<Eigen::Vector2f> buffer) {
        Eigen::Vector2f point = buffer[0];
        cluster_lines[index_of_cluster].mean_value.push_back(MeanValue(0, point));

        for(int i = 1; i < buffer.size(); i++) {
            point = buffer[i];
            cluster_lines[index_of_cluster].mean_value.push_back(MeanValue(i, point));
            cluster_lines[index_of_cluster].covariance_matrix = CovarianceMatrix(i, point);
        }

        cluster_lines[index_of_cluster].normal_vector = NormalVector();
        for(int i = 1; i < buffer.size(); i++) {
            cluster_lines[index_of_cluster].orthogonal_distance = OrthogonalDistance(point);
            cluster_lines[index_of_cluster].cluster_distance_variance = ClusterdistanceVariance(i);
        }

        // std::cout << "<==============================" << std::endl;
        // std::cout << "point:\n" << point << std::endl;
        // std::cout << "mean:\n"  << cluster_lines[index_of_cluster].mean_value[cluster_lines[index_of_cluster].mean_value.size() - 1] << std::endl;
        // std::cout << "covariance:\n" << cluster_lines[index_of_cluster].covariance_matrix << std::endl;
        // std::cout << "normal:\n" << cluster_lines[index_of_cluster].normal_vector << std::endl;
        // std::cout << "orthogonal: " << cluster_lines[index_of_cluster].orthogonal_distance << std::endl;
        // std::cout << "variance: " << cluster_lines[index_of_cluster].cluster_distance_variance << std::endl;
        // std::cout << "==============================>" << std::endl;

    }

    Eigen::Vector2f LineClustering::MeanValue(int index, Eigen::Vector2f point) {
        Eigen::Vector2f matrix_u;

        Line line = cluster_lines[index_of_cluster];
        float k = index + 1;
        if (k == 1) {
            matrix_u = point;
        } else {
            matrix_u = ((k - 1) / k) * line.mean_value[line.mean_value.size() - 1] + (1 / k) * point;
        }

        return matrix_u;   
    }

    Eigen::Matrix2f LineClustering::CovarianceMatrix(int index, Eigen::Vector2f point) {
        Eigen::Matrix2f matrix_cov;

        // not k == 1
        float k = index + 1;
        Line line = cluster_lines[index_of_cluster];
        if(k == 2) {
            matrix_cov = (1 / k) * (point - line.mean_value[index - 1]) * (point - line.mean_value[index - 1]).transpose();
        } else {
            matrix_cov = ((k - 2) / (k - 1)) * line.covariance_matrix + (1 / k) * (point - line.mean_value[index - 1]) * (point - line.mean_value[index - 1]).transpose();
        }

        return matrix_cov;
    }

    Eigen::Vector2f LineClustering::NormalVector() {
        Eigen::Vector2f nor_vec;

        Line line = cluster_lines[index_of_cluster];
        float th = line.covariance_matrix(0, 1) / line.covariance_matrix(0, 0);
        nor_vec(0) = -(th / sqrtf(pow(th, 2) + 1));
        nor_vec(1) = 1 / sqrtf(pow(th, 2) + 1);

        return nor_vec;
    }

    float LineClustering::OrthogonalDistance(Eigen::Vector2f point) {
        float od;
        Line line = cluster_lines[index_of_cluster];
        od = fabsf((point - line.mean_value[line.mean_value.size() - 1]).transpose() * line.normal_vector);

        return od;
    }

    float LineClustering::ClusterdistanceVariance(int index) {
        float cdv;

        float k = index + 1;
        Line line = cluster_lines[index_of_cluster];
        if(k == 2) {
            cdv = (1 / k) * pow(line.orthogonal_distance, 2);
        } else {
            cdv = line.cluster_distance_variance * ((k - 2) / (k - 1)) + (1 / k) * pow(line.orthogonal_distance, 2); 
        }

        return cdv;
    }

    void LineClustering::DetermineCorner() {
        // ROS_INFO("Determining Corner point");
        if(cluster_lines.size() > 1) {
            for(int i = 0; i < index_of_cluster; i++) {
                Line line_1 = cluster_lines[i];
                Line line_2 = cluster_lines[i + 1];

                std::vector<float> p_1_1 = {line_1.points_in_line[0](0, 0), line_1.points_in_line[0](1, 0)};            
                std::vector<float> p_1_2 = {line_1.points_in_line[line_1.points_in_line.size() - 1](0, 0), line_1.points_in_line[line_1.points_in_line.size() - 1](1, 0)};

                std::vector<float> p_2_1 = {line_2.points_in_line[0](0, 0), line_2.points_in_line[0](1, 0)};
                std::vector<float> p_2_2 = {line_2.points_in_line[line_2.points_in_line.size() - 1](0, 0), line_2.points_in_line[line_2.points_in_line.size() - 1](1, 0)};

                float dis = VectorUtils::GetTwoPointsDistance(p_1_2, p_2_1);

                std::vector<float> v1 = {p_1_2[0] - p_1_1[0], p_1_2[1] - p_1_1[1]};
                std::vector<float> v2 = {p_2_2[0] - p_2_1[0], p_2_2[1] - p_2_1[1]};
                float theta = acosf((v1[0] * v2[0] + v1[1] * v2[1]) / ((sqrtf(pow(v1[0], 2) + pow(v1[1], 2))) * (sqrtf(pow(v2[0], 2) + pow(v2[1], 2))))) * 180 / M_PI;
                // Angle Threshold
                if((theta > 70 && theta < 110) && dis < 0.2) {
                    if(i - 1 >= 0 && i + 2 <= index_of_cluster) {
                        Line line_0 = cluster_lines[i - 1];
                        Line line_3 = cluster_lines[i + 2];

                        std::vector<float> p_0_1 = {line_0.points_in_line[0](0, 0), line_0.points_in_line[0](1, 0)};            
                        std::vector<float> p_0_2 = {line_0.points_in_line[line_0.points_in_line.size() - 1](0, 0), line_0.points_in_line[line_0.points_in_line.size() - 1](1, 0)};

                        std::vector<float> p_3_1 = {line_3.points_in_line[0](0, 0), line_3.points_in_line[0](1, 0)};
                        std::vector<float> p_3_2 = {line_3.points_in_line[line_3.points_in_line.size() - 1](0, 0), line_3.points_in_line[line_3.points_in_line.size() - 1](1, 0)};

                    }

                    // std::cout << i << "CORNER : " << theta << ", DIS : " << dis << std::endl;
                    std::vector<float> eq_1 = VectorUtils::GetEquationParameter(p_1_1, p_1_2);
                    std::vector<float> eq_2 = VectorUtils::GetEquationParameter(p_2_1, p_2_2);
                    float px = (eq_1[1] * eq_2[2] - eq_2[1] * eq_1[2]) / (eq_1[0] * eq_2[1] - eq_2[0] * eq_1[1]);
                    float py = -(eq_1[0] / eq_1[1]) * (px) - (eq_1[2] / eq_1[1]);
                    std::vector<float> p = {px, py};
                    corner_points.push_back(p);

                    temp.push_back(p_1_1);
                    temp.push_back(p_1_2);
                    temp.push_back(p_2_1);
                    temp.push_back(p_2_2);
                }
            }
        }
    }

    float LineClustering::GetTwoLineAngle(Line line1, Line line2) {
        std::vector<float> p_1_1 = {line1.points_in_line[0](0, 0), line1.points_in_line[0](1, 0)};
        std::vector<float> p_1_2 = {line1.points_in_line[line1.points_in_line.size() - 1](0, 0), line1.points_in_line[line1.points_in_line.size() - 1](1, 0)};

        std::vector<float> p_2_1 = {line2.points_in_line[0](0, 0), line2.points_in_line[0](1, 0)};
        std::vector<float> p_2_2 = {line2.points_in_line[line2.points_in_line.size() - 1](0, 0), line2.points_in_line[line2.points_in_line.size() - 1](1, 0)};

        std::vector<float> v1 = {p_1_2[0] - p_1_1[0], p_1_2[1] - p_1_1[1]};
        std::vector<float> v2 = {p_2_2[0] - p_2_1[0], p_2_2[1] - p_2_1[1]};
        float theta = acosf((v1[0] * v2[0] + v1[1] * v2[1]) / ((sqrtf(pow(v1[0], 2) + pow(v1[1], 2))) * (sqrtf(pow(v2[0], 2) + pow(v2[1], 2))))) * 180 / M_PI;

        return theta;
    }
}