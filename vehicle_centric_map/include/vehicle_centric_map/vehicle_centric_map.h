#ifndef _VEHICLE_CENTRIC_MAP_H
#define _VEHICLE_CENTRIC_MAP_H

#include <iostream>
#include <random>
#include <queue>
#include <tuple>
#include <chrono>
#include <mutex>
#include <assert.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <vehicle_centric_map/raycast.h>

#define PI 3.1415926
#define logit(x) (log((x) / (1 - (x))))

using namespace std;

class VehicleCentriMap{
    public:
        VehicleCentriMap(ros::NodeHandle &nh);
        ~VehicleCentriMap(){};

    //API for trajectory planning and optimization
    double evaluateCoarseEDT(const Eigen::Vector2d&pos);
    void evaluateEDTWithGrad(const Eigen::Vector2d&pos, double& dist,Eigen::Vector3d&grad);
    int getInflateOccupancy(Eigen::Vector2d pos);
    void setOrigin(Eigen::Vector3d &ori, Eigen::Vector3d& size);
    bool checkMapInit(){return map_init_;}

    typedef std::shared_ptr<VehicleCentriMap> Ptr;
        /* ros things */
    ros::NodeHandle node_;
    ros::Subscriber nav_map_sub_;
    ros::Publisher esdf_pub_, map_inf_pub_;
    ros::Timer vis_timer_;


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    std::mutex mtx;
    /* == Mapping Data == */
    std::vector<char> occupancy_grid_inflate_;
    std::vector<char> occupancy_grid_neg;
    std::vector<double> distance_buffer_;
    std::vector<double> distance_buffer_neg_;
    std::vector<double> distance_buffer_all_;
    std::vector<double> tmp_buffer1_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inflated_cloud_to_pub;

    /* == Mapping Parameters == */
    bool map_init_;
    double map_origin_x;
    double map_origin_y;
    double map_length;
    double map_resolution_,map_resolution_inv_;
    int map_grid_num;
    int map_length_grid;
    double obstacles_inflation;
    int p_occ;

    /* == main loop == */
    void occupancyCallback(const nav_msgs::OccupancyGridConstPtr& msg);

   /*  operation in the main loop */
    void init(const int map_width,const int map_height, const double resolution);
    void clearAndInflateLocalMap(const nav_msgs::OccupancyGrid& map);
    void updateESDF();
    /* tools */

    inline void inflatePoint(const Eigen::Vector2i& pt, int step, vector<int>& pts);
    void interpolateBilinear(double values[2][2],const Eigen::Vector2d &diff,double& value,Eigen::Vector3d&grad);
    void getSurroundPts(const Eigen::Vector2d& pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d& diff);
    void getSurroundDistance(Eigen::Vector2d pts[2][2],double dists[2][2]);
    inline double getDistance(const Eigen::Vector2d& pos);
    inline bool isInMap(const Eigen::Vector2d &pos);

    template <typename F_get_val, typename F_set_val>
    void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

    /* index calculation */
    inline void boundIndex(int& id) ;

    inline Eigen::Vector2i pos2LocalIndex(const Eigen::Vector2d &pos);
    inline void localIndex2Pos(const Eigen::Vector2i&id,Eigen::Vector2d &pos);

    /* visualization */
    void publishESDF();
    void publishMap();


};



#endif
