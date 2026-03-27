#ifndef _DYNAMIC_OCCUPANCY_GRID_H
#define _DYNAMIC_OCCUPANCY_GRID_H

#include <iostream>
#include <random>
#include <queue>
#include <tuple>
#include <chrono>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <dogm/dogm.h>
#include <dogm/dogm_types.h>
#include  <dogm/mapping/laser_to_meas_grid.h>
#include <dogm/utils/image_creation.h>

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <raycast.h>

#define PI 3.1415926
#define logit(x) (log((x) / (1 - (x))))

using namespace std;
// #define RAYCAST
//2D mapping data

class DynamicOccupancyGrid{
    public:
        DynamicOccupancyGrid(ros::NodeHandle &nh);
        ~DynamicOccupancyGrid(){};


    typedef std::shared_ptr<DynamicOccupancyGrid> Ptr;
        /* ros things */
    ros::NodeHandle node_;
    ros::Publisher nav_map_pub_;
    ros::Subscriber collision_point_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>
    SyncPolicyCloudOdom;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;
    shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
    shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
    SynchronizerCloudOdom sync_cloud_odom_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
    /* == Mapping Data == */
    std::vector<float> measurements;
    nav_msgs::OccupancyGrid ros_map;
    std::vector<double> occupancy_grid_;

    /* == Mapping Parameters == */
    double map_resolution_,map_resolution_inv_;
    double map_length,map_length_half;
    int map_length_grid;

    /* == Sensor Parameters == */
    double max_range;
    int horizontal_scan_num;
    double angle_increment;
    double angle_max,angle_min;
    double sensor_height_limit_upper,sensor_height_limit_down;
    double sensor_pitch;
    std_msgs::Header cloud_header;
    double dt=-1.0;
    bool if_cloud_need_transform;

    /* == PHD/MIB-filter == */
    dogm::DOGM::Params params_;
	LaserMeasurementGrid::Params laser_params_;
	std::unique_ptr<LaserMeasurementGrid> grid_generator_;
    std::unique_ptr<dogm::DOGM> grid_map_;

    /* == raycasting ==*/
    // occupancy probability
    double p_hit_, p_miss_, p_min_, p_max_, p_occ_; 
    // logit of occupancy probability
    double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,min_occupancy_log_;
    // range of doing raycasting
    double min_ray_length_, max_ray_length_; 
    // flag buffers for speeding up raycasting
    std::vector<short> count_hit_, count_hit_and_miss_;

    char raycast_num_;
    std::queue<Eigen::Vector2i> cache_voxel_;

    /* probability map */
    

    /* active mapping */
    double unknown_flag_;

    /* Prediction */
    bool if_predict_need;
    geometry_msgs::Point collision_point;
    float minVelocityThres;
    float dt_predict;

    /* visualization */
    pcl::PointCloud<pcl::PointXYZ>::Ptr inflated_cloud_to_pub;
    pcl::VoxelGrid<pcl::PointXYZ> downSampleCloud;
    pcl::PassThrough<pcl::PointXYZ> passThroughCloud;

    /* == main loop == */
    void cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& msg_cloud,const nav_msgs::OdometryConstPtr& msg_odom);
    void collisionPointCallback(const geometry_msgs::PointConstPtr& msg);
    // void test();

   /*  operation in the main loop */
    void project2ScanMeasurements(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudIn);
    void egoMotionCompensation(const geometry_msgs::Point & position);
    void raycastUpdate();
    void dogmUpdate();
    void addPredictionLayer(const std::vector<dogm::GridCell> &grid_cell_array);
    /* tools */

    void setCacheOccupancy(Eigen::Vector2d pos, bool occ);

    /* index calculation */
    inline void boundIndex(int& id) ;
    inline void boundAngleIndex(int& id);

    inline int toAddress(Eigen::Vector2i id_local_2d);
};



#endif
