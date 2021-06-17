#include "RobotState.hpp"
//
namespace CornerDetector {
    // TODO: 3d 채널로 뽑기
    RobotState::RobotState() {
        SetParameter();
        marker_EPCC = n.advertise<visualization_msgs::Marker>("EPCC", 10);
        marker_EPCC_corner = n.advertise<visualization_msgs::Marker>("EPCC_corner", 10);
        corner_rt = n.advertise<mrpt_msgs::ObservationRangeBearing>("corner", 10);

        std::thread t1(&RobotState::TFUpdate, this);
        Run();
    }

    RobotState::~RobotState() {

    }

    void RobotState::Run() {
        if(use_2d_lidar) {
            lidar_data = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, &RobotState::LaserScanCallback, this);
        } else {
            lidar_data = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &RobotState::PointCloud2Callback, this);
        }
        while(ros::ok()) {
            ros::spin();
        }
    }

    void RobotState::SetParameter() {
        n.getParam("/use_2d_lidar", use_2d_lidar);
    }

    void RobotState::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        laser_geometry::LaserProjection projector_;
        projector_.projectLaser(*msg, laserscan);

        downsampling_data = LaserScanToPoint();

        CornerDetector::LineClustering line_clustering(downsampling_data);
        line_clustering.EPCC();
        line_clustering.DetermineCorner();

        geometry_msgs::Pose robot_tf_ = robot_tf;
        // ACCUMULATE FEATURES
        accumulate_features.Run(line_clustering.corner_points,
                                robot_tf,
                                robot_velodyne_tf_x,
                                robot_velodyne_tf_y);

        // VISUALIZATION
        visualization_msgs::Marker corner;
        CornerDetector::Visualization vs;
        vs.vis(accumulate_features.old_f, corner);
        marker_EPCC_corner.publish(corner);

        // Math.atan2(deltaY, deltaX)
        mrpt_msgs::ObservationRangeBearing points;
        points.header.frame_id = "base_scan";
        for (AccumulateFeatures::Features features:accumulate_features.old_f) {
            std::vector<float> xy = {robot_tf_.position.x, robot_tf_.position.y};
            float r = VectorUtils::GetTwoPointsDistance(xy, features.f);

            tf::Quaternion q(
                robot_tf_.orientation.x,
                robot_tf_.orientation.y,
                robot_tf_.orientation.z,
                robot_tf_.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            double theta = std::atan2(features.f[1] - xy[1], features.f[0] - xy[0]);
            std::cout << yaw << ", " << theta << std::endl;

            mrpt_msgs::SingleRangeBearingObservation p;
            p.range = r;
            p.yaw = theta + yaw;

            // Not use
            p.pitch = 0;
            p.id = 0;

            points.sensed_data.push_back(p);
        }
        corner_rt.publish(points);
    }

    void RobotState::PointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::fromROSMsg(*msg, pointcloud);

        downsampling_data = GatherSpecificRingPoints();
    }

    std::vector<Eigen::Vector2f> RobotState::GatherSpecificRingPoints() {
        std::vector<Eigen::Vector2f> downsampling;
        std::vector<float> v_center = {0.0, 0.0};
        std::vector<float> pre_point;    
        std::vector<float> curr_point;

        BOOST_FOREACH(const velodyne_pointcloud::PointXYZIR point, pointcloud.points) {
            if(point.ring == ring) {
                curr_point = {point.x, point.y};
                float center_to_point_distance = VectorUtils::GetTwoPointsDistance(v_center, curr_point);
                if(min_distance <= center_to_point_distance && max_distance >= center_to_point_distance) {
                    if(downsampling.size() == 0) {
                        Eigen::Vector2f p(point.x, point.y);
                        downsampling.push_back(p);
                        pre_point = curr_point;
                    } else {
                        float distance_two_points = VectorUtils::GetTwoPointsDistance(pre_point, curr_point);
                        if(downsampling_threshold < distance_two_points) {
                            Eigen::Vector2f p(point.x, point.y);
                            downsampling.push_back(p);
                            pre_point = curr_point;
                        }
                    }
                }
            }
        }
        return downsampling;
    }

    std::vector<Eigen::Vector2f> RobotState::LaserScanToPoint() {
        std::vector<Eigen::Vector2f> downsampling;
        std::vector<float> v_center = {0.0, 0.0};
        std::vector<float> pre_point;    
        std::vector<float> curr_point;

        BOOST_FOREACH(const geometry_msgs::Point32 point, laserscan.points) {
            curr_point = {point.x, point.y};
            float center_to_point_distance = VectorUtils::GetTwoPointsDistance(v_center, curr_point);
            if(min_distance <= center_to_point_distance && max_distance >= center_to_point_distance) {
                if(downsampling.size() == 0) {
                    Eigen::Vector2f p(point.x, point.y);
                    downsampling.push_back(p);
                    pre_point = curr_point;
                } else {
                    float distance_two_points = VectorUtils::GetTwoPointsDistance(pre_point, curr_point);
                    if(downsampling_threshold < distance_two_points) {
                        Eigen::Vector2f p(point.x, point.y);
                        downsampling.push_back(p);
                        pre_point = curr_point;
                    }
                }
            }
        }
        return downsampling;
    }

    void RobotState::TFUpdate() {
        while(ros::ok()) {
            tf::TransformListener tf;
            tf::StampedTransform transform_robot;
            try{
                tf.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
                tf.lookupTransform("map", "base_link", ros::Time(0), transform_robot);
                
                robot_tf.position.x = transform_robot.getOrigin().getX();
                robot_tf.position.y = transform_robot.getOrigin().getY();
                robot_tf.position.z = transform_robot.getOrigin().getZ();
                robot_tf.orientation.z = tf::getYaw(transform_robot.getRotation());

                tf_initialization = true;
            } catch (tf::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }
    }
}   
