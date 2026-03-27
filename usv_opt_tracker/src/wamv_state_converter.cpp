#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <Eigen/Eigen>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

class WamvStateConverter
{
public:
  WamvStateConverter()
  {
    // 订阅gazebo_msgs/ModelStates话题
    model_states_sub_ = nh_.subscribe("/gazebo/model_states", 10, &WamvStateConverter::modelStatesCallback, this,
                                      ros::TransportHints().tcpNoDelay());
    // 发布nav_msgs/Odometry话题
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/wamv/odom", 10);

    rc_override_sub_ = nh_.subscribe("/mavros/rc/override", 1, &WamvStateConverter::rcOverrideCallback, this,
                                     ros::TransportHints().tcpNoDelay());
    // 发布std_msgs/Float32话题
    left_thrust_pub_ = nh_.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 10);
    right_thrust_pub_ = nh_.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 10);
  }

private:
  void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
      // 获取“wamv”的姿态和速度信息
      geometry_msgs::Pose pose = msg->pose[17];
      geometry_msgs::Twist twist = msg->twist[17];

      // 创建并填充nav_msgs/Odometry消息
      nav_msgs::Odometry odom;
      odom.header.stamp = ros::Time::now();
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
      odom.pose.pose = pose;
      odom.twist.twist = twist;

      // Transform linear velocity to body frame
      tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
                       odom.pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      Eigen::Matrix3d R;
      R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

      Eigen::Vector3d vel_world(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);

      Eigen::Vector3d vel_body = R.transpose() * vel_world;

      odom.twist.twist.linear.x = vel_body.x();
      odom.twist.twist.linear.y = vel_body.y();
      odom.twist.twist.linear.z = vel_body.z();

      // 发布odom消息
      odom_pub_.publish(odom);
      //发布tf变换
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
  }

  void rcOverrideCallback(const mavros_msgs::OverrideRCIn::ConstPtr& msg)
  {
    // 将RC Override消息转换为左、右推力命令
    std_msgs::Float32 left_thrust_cmd;
    std_msgs::Float32 right_thrust_cmd;

    // 假设通道1和通道3分别对应左、右推力命令
    left_thrust_cmd.data = (static_cast<float>(msg->channels[0])-1500.f)/500.f;
    right_thrust_cmd.data = (static_cast<float>(msg->channels[2])-1500.f)/500.f;

    // 发布推力命令
    left_thrust_pub_.publish(left_thrust_cmd);
    right_thrust_pub_.publish(right_thrust_cmd);
  }

  ros::NodeHandle nh_;
  ros::Subscriber model_states_sub_;
  ros::Subscriber rc_override_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher left_thrust_pub_;
  ros::Publisher right_thrust_pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wamv_state_converter");
  WamvStateConverter wamv_state_converter;
  ros::spin();
  return 0;
}