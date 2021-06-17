#ifndef ROBOT_STATE_HEADER_
#define ROBOT_STATE_HEADER_

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/pointcloudXYZIR.h>
#include <velodyne_pointcloud/convert.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2_msgs/TFMessage.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"
#include <tf2_ros/transform_listener.h>
#include "tf/transform_listener.h"
#include "boost/foreach.hpp"
#include "string"
#include "thread"
#include "future"
#include "Eigen/Dense"
#include "vector"
#include "math.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include <unistd.h>
#include "mrpt_msgs/ObservationRangeBearing.h"

#include "LineClustering.hpp"
#include "VectorUtils.hpp"
#include "Visualization.hpp"
#include "GlobalFeatureMap.hpp"
#include "PointMatching.hpp"
#include "LineMatching.hpp"
#include "AccumulateFeatures.hpp"

namespace CornerDetector
{
class RobotState {
    private:
        bool use_2d_lidar;

        // ros
        ros::NodeHandle n;
        ros::Subscriber lidar_data;

        float robot_velodyne_tf_x = 0.23;
        float robot_velodyne_tf_y = 0;

        // lidar data
        float downsampling_threshold = 0.05;
        float min_distance = 0.5;
        float max_distance = 10;
        int ring = 8;
        std::vector<Eigen::Vector2f> downsampling_data;
    
        // 3d point cloud
        pcl::PointCloud<velodyne_pointcloud::PointXYZIR> pointcloud;
        // 2d laser scan
        sensor_msgs::PointCloud laserscan;
        // tf
        bool tf_initialization = false;
        geometry_msgs::Pose robot_tf;

        // globalpoint
        CornerDetector::GlobalFeatureMap gm;
        Eigen::MatrixXf m;

        CornerDetector::AccumulateFeatures accumulate_features;

        // publish point
        ros::Publisher marker_EPCC;
        // ros::Publisher marker_downsampling;
        ros::Publisher marker_EPCC_corner;
        // ros::Publisher marker_EPCC_tmp;
        ros::Publisher posearray_corner_point;

        ros::Publisher corner_rt;

        void Run();
        void SetParameter();
        void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void PointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

        std::vector<Eigen::Vector2f> GatherSpecificRingPoints();
        // void GatherSpecificRingPoints_3DVersion(const sensor_msgs::PointCloud2::ConstPtr &input_cloud);

        std::vector<Eigen::Vector2f> LaserScanToPoint();

        void TFUpdate();
    public:
        RobotState();
        ~RobotState();
};
}

#endif
