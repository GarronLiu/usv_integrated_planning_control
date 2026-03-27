#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPoseStamped.h>

#include <GeographicLib/Geocentric.hpp>
#include <geographic_msgs/GeoPointStamped.h>

#include <Eigen/Eigen>

#include <iostream>
using namespace std;


Eigen::Vector3d map_origin {};	//!< oigin of map frame [lla]
Eigen::Vector3d ecef_origin {};	//!< geocentric origin [m]
bool is_map_init = false;
ros::Time stamp_;
ros::Publisher pub;
void gp_origin_cb(const geographic_msgs::GeoPointStamped::ConstPtr &msg)
	{
		stamp_ = msg->header.stamp;
		// ecef_origin = {msg->position.latitude, msg->position.longitude, msg->position.altitude};
		// /**
		//  * @brief Conversion from ECEF (Earth-Centered, Earth-Fixed) to geodetic coordinates (LLA)
		// */
		// GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
		// try {
		// 	earth.Reverse(ecef_origin.x(), ecef_origin.y(), ecef_origin.z(),
		// 		map_origin.x(), map_origin.y(), map_origin.z());
		// }
		// catch (const std::exception& e) {
		// 	ROS_WARN_STREAM("setpoint: Caught exception: " << e.what() << std::endl);
		// 	return;
		// }
		map_origin.x() = msg->position.latitude;
		map_origin.y() = msg->position.longitude;
		map_origin.z() = msg->position.altitude;
        ROS_INFO("Received GPS_GLOBAL_ORIGIN: Latitude=%.9f, Longitude=%.9f, Altitude=%.9f", map_origin.x(), map_origin.y(), map_origin.z() );
		is_map_init = true;
	}

void ekfOriginTimer(const ros::TimerEvent&){
	if(is_map_init){
		sensor_msgs::NavSatFix msg;
			msg.header.frame_id = "odom";
			msg.header.stamp = stamp_;
			msg.latitude = map_origin.x();
			msg.longitude = map_origin.y();
			msg.altitude = map_origin.z();

		pub.publish(msg);
	}
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ekf_origin_acquire");
    ros::NodeHandle nh;

    // Subscribe to the MAVLink messages
    ros::Subscriber sub = nh.subscribe("/mavros/global_position/gp_origin", 10, gp_origin_cb);
	pub = nh.advertise<sensor_msgs::NavSatFix>("/mavros/global_position/ekf_origin", 10);
    ros::Timer timer = nh.createTimer(ros::Duration(1),&ekfOriginTimer);

    ros::spin();

    return 0;
}
