#include <ros/ros.h>
#include <dynamic_occupancy_grid.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dogm_node");
	ROS_INFO("\033[1;32m----> Dynamic Occupancy Grid Map Started.\033[0m");
    ros::NodeHandle node;
    DynamicOccupancyGrid vcm(node);
	ros::spin();

	return 0;
}