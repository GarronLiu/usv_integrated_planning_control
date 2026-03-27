// Main Contributor:Garron Liu
// Date: 16th Dec. 2023

#ifndef USV_PATH_GENERATOR_H
#define USV_PATH_GENERATOR_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Eigen>

#include <GeographicLib/LocalCartesian.hpp>

#include <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <cassert>

using namespace std;

/// FUTURE:1.解决目前bspline拟合路径可能存在的震荡问题,针对手工标点生成轨迹的方式进行改进，考虑加减速的过程
// 2.订阅路径规划的轨迹并执行
// 3.添加nmpc求解器
class Usv_Path_Generator
{
private:
  nav_msgs::Odometry cur_odom;

  // reference trajectory related
  std::vector<Eigen::Vector3d> path_tracking_vec;
  double vel_avg, vel_max;

  nav_msgs::Path pathRaw;
  nav_msgs::Path pathSmooth;
  nav_msgs::Path pathReal;

  double path_res, min_turn_radius;
  Eigen::Vector3d last_arc_to_line_point;
  double control_time_step = 0.05;


  // lock线程锁
  std::mutex traj_mtx;
  std::mutex state_mtx;
  std::mutex rc_out_mtx;
  std::mutex rc_out_msgs_mtx;
  std::mutex rc_in_mtx;

  visualization_msgs::Marker mk_history_points;

public:
  ros::NodeHandle node_;
  ros::Subscriber odom_sub_;
  ros::Subscriber nav_point_sub_;

  ros::Publisher smooth_path_pub_;
  ros::Publisher traj_all_pub_, path_point_pub_;
  ros::Publisher traj_predict_pub_;

  ros::Timer control_timer_;

  bool odom_available;

  Usv_Path_Generator(ros::NodeHandle& n)
  {
    node_ = n;
    node_.param<double>("/path_generator/MaxVel", vel_max, 2.0);
    node_.param<double>("/path_generator/PathResolution", path_res, 0.5);
    node_.param<double>("/path_generator/MinTurnRadius", min_turn_radius, 2.0);

    nav_point_sub_ = node_.subscribe<geometry_msgs::PoseStamped>("/goal", 100, &Usv_Path_Generator::goal_cbk, this);

    odom_sub_ = node_.subscribe<nav_msgs::Odometry>("/odometry", 1000, &Usv_Path_Generator::odom_cbk, this,
                                                    ros::TransportHints().tcpNoDelay());
    odom_available = false;
    traj_all_pub_ = node_.advertise<nav_msgs::Path>("/manager/trajectory/tracking", 1000);
    path_point_pub_ = node_.advertise<visualization_msgs::Marker>("path_generator/trajectory/knot", 50);
    pathReal.header.frame_id = "odom";
  }

  /* 手工标记跟踪点的回调函数 */
  void goal_cbk(const geometry_msgs::PoseStampedConstPtr& goal_in)
  {
    if (!odom_available)
      return;

    Eigen::Vector3d tracking_point;
    tracking_point.x() = goal_in->pose.position.x;
    tracking_point.y() = goal_in->pose.position.y;
    tracking_point.z() = 0;

    std::lock_guard<std::mutex> lock(traj_mtx);
    if (path_tracking_vec.empty())
    {
      Eigen::Vector3f start_point;
      Eigen::Quaternionf start_quat;
      {
        std::lock_guard<std::mutex> lck(state_mtx);
        start_point = Eigen::Vector3f(cur_odom.pose.pose.position.x, cur_odom.pose.pose.position.y,
                                      cur_odom.pose.pose.position.z);
        start_quat.w() = cur_odom.pose.pose.orientation.w;
        start_quat.x() = cur_odom.pose.pose.orientation.x;
        start_quat.y() = cur_odom.pose.pose.orientation.y;
        start_quat.z() = cur_odom.pose.pose.orientation.z;
      }
      last_arc_to_line_point = Eigen::Vector3d(start_point(0), start_point(1), 0);
      path_tracking_vec.push_back(last_arc_to_line_point);
      auto temp = start_quat * Eigen::Vector3f(5.0, 0, 0);
      discretizePath(path_tracking_vec, last_arc_to_line_point,
                     Eigen::Vector3d(temp(0), temp(1), 0) + last_arc_to_line_point, path_res);
    }

    //圆弧插补
    Eigen::Vector3d intersect_point = path_tracking_vec.back();
    Eigen::Vector3d last_direct_ = (intersect_point - last_arc_to_line_point).normalized();
    Eigen::Vector3d next_direct_ = (tracking_point - intersect_point).normalized();

    double angleBetween = acos(last_direct_.dot(next_direct_));
    if (angleBetween * min_turn_radius < path_res)
    {
      discretizePath(path_tracking_vec, path_tracking_vec.back(), tracking_point, path_res);
    }
    else
    {
      // 计算切点
      double tangent_to_intersect_dist = min_turn_radius * tan(angleBetween / 2);
      if ((last_arc_to_line_point - intersect_point).norm() < tangent_to_intersect_dist + path_res)
      {
        ROS_WARN("\033[1;33mTurning Angle is too large, please reset!\033[0m");
        return;
      }

      if ((tracking_point - intersect_point).norm() < tangent_to_intersect_dist + path_res)
      {
        ROS_WARN("\033[1;33mNew Tracking point is too near, please reset!\033[0m");
        return;
      }

      Eigen::Vector3d tangent_point_start = intersect_point - last_direct_ * tangent_to_intersect_dist;
      size_t id_ = path_tracking_vec.size() - 1;
      while ((path_tracking_vec[id_] - intersect_point).norm() < (tangent_to_intersect_dist + path_res))
      {
        path_tracking_vec.pop_back();
        id_--;
      }
      if (id_ == 0)
      {
        ROS_WARN("Some error exsist!");
        return;
      }

      Eigen::Vector3d cross_vec_ = last_direct_.cross(next_direct_);
      //计算法向量
      double rot_sign = (cross_vec_(2) > 0.0) ? (1.0) : (-1.0);
      double normal_vec_angle = atan2(last_direct_(1), last_direct_(0)) + rot_sign * M_PI_2;
      Eigen::Vector3d normal_vec_(cos(normal_vec_angle), sin(normal_vec_angle), 0);
      //计算插补圆的圆心
      Eigen::Vector3d arc_center = tangent_point_start + min_turn_radius * normal_vec_;

      double angle_step_ = rot_sign * path_res / min_turn_radius;
      double angle_begin_ = normal_vec_angle + M_PI;
      double discrete_angle = 0;
      angleBetween -= abs(angle_step_);
      while (discrete_angle < angleBetween && discrete_angle > -angleBetween)
      {
        Eigen::Vector3d pt_insert =
            arc_center + min_turn_radius *
                             Eigen::Vector3d(cos(discrete_angle + angle_begin_), sin(discrete_angle + angle_begin_), 0);
        path_tracking_vec.push_back(pt_insert);
        discrete_angle += angle_step_;
      }

      last_arc_to_line_point = intersect_point + next_direct_ * tangent_to_intersect_dist;
      path_tracking_vec.push_back(last_arc_to_line_point);
      discretizePath(path_tracking_vec, last_arc_to_line_point, tracking_point, path_res);
    }

    visDiscretTrajPoints();
    parametrizeTraj();
  }

  void discretizePath(std::vector<Eigen::Vector3d>& path_vec, const Eigen::Vector3d& p_start,
                      const Eigen::Vector3d& p_end, double res)
  {
    Eigen::Vector3d direct_ = p_end - p_start;
    float direct_norm_ = direct_.norm();
    if (direct_norm_ < res)
    {
      printf("Tracking point manually set too closed, please reset!");
      return;
    }
    int discret_num = round(direct_norm_ / res);
    direct_ /= direct_norm_;
    for (int i = 1; i <= discret_num; i++)
    {
      Eigen::Vector3d pt = i * res * direct_ + p_start;
      path_vec.push_back(pt);
    }
  }

  geometry_msgs::PoseStamped createPoseStamped(double x, double y, double z)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    return pose;
  }

  /* 参数化轨迹 */
  void parametrizeTraj()
  {
    pathRaw = nav_msgs::Path();
    // traj_mtx.lock();
    for (size_t i = 0; i < path_tracking_vec.size(); i++)
    {
      pathRaw.poses.push_back(
          pos2PoseStamped(Eigen::Vector3f(path_tracking_vec[i].x(), path_tracking_vec[i].y(), 0.0f)));
    }
    // traj_mtx.unlock();
    pathSmooth = nav_msgs::Path();
    pathSmooth = processPath(pathRaw);

    pathSmooth.header.frame_id = "odom";
    pathSmooth.header.stamp = ros::Time::now();
    traj_all_pub_.publish(pathSmooth);
  }

  nav_msgs::Path processPath(nav_msgs::Path pathIn)
  {
    pathIn = calculatePathYaw(pathIn);
    pathIn = fixPathDensity(pathIn);
    pathIn = smoothPath(pathIn);
    return pathIn;
  }

  nav_msgs::Path calculatePathYaw(nav_msgs::Path pathIn)
  {
    int length = pathIn.poses.size();
    if (length <= 1)
    {
      if (length == 1)
        pathIn.poses[0].pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
      return pathIn;
    }

    for (int i = 0; i < length - 1; ++i)
    {
      double dx = pathIn.poses[i + 1].pose.position.x - pathIn.poses[i].pose.position.x;
      double dy = pathIn.poses[i + 1].pose.position.y - pathIn.poses[i].pose.position.y;
      double theta = atan2(dy, dx);
      pathIn.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    }

    pathIn.poses.back().pose.orientation = pathIn.poses[length - 2].pose.orientation;

    return pathIn;
  }

  nav_msgs::Path smoothPath(nav_msgs::Path path)
  {
    if (path.poses.size() <= 2)
      return path;

    double weight_data = 0.45;
    double weight_smooth = 0.4;
    double tolerance = 0.05;

    nav_msgs::Path smoothPath_out = path;

    double change = tolerance;
    double xtemp, ytemp;
    int nIterations = 0;

    int size = path.poses.size();

    while (change >= tolerance)
    {
      change = 0.0;
      for (int i = 1; i < size - 1; i++)
      {
        xtemp = smoothPath_out.poses[i].pose.position.x;
        ytemp = smoothPath_out.poses[i].pose.position.y;

        smoothPath_out.poses[i].pose.position.x +=
            weight_data * (path.poses[i].pose.position.x - smoothPath_out.poses[i].pose.position.x);
        smoothPath_out.poses[i].pose.position.y +=
            weight_data * (path.poses[i].pose.position.y - smoothPath_out.poses[i].pose.position.y);

        smoothPath_out.poses[i].pose.position.x +=
            weight_smooth * (smoothPath_out.poses[i - 1].pose.position.x + smoothPath_out.poses[i + 1].pose.position.x -
                             (2.0 * smoothPath_out.poses[i].pose.position.x));
        smoothPath_out.poses[i].pose.position.y +=
            weight_smooth * (smoothPath_out.poses[i - 1].pose.position.y + smoothPath_out.poses[i + 1].pose.position.y -
                             (2.0 * smoothPath_out.poses[i].pose.position.y));

        change += fabs(xtemp - smoothPath_out.poses[i].pose.position.x);
        change += fabs(ytemp - smoothPath_out.poses[i].pose.position.y);
      }
      nIterations++;
    }

    return smoothPath_out;
  }

  nav_msgs::Path fixPathDensity(nav_msgs::Path path)
  {
    double _pathResolution = vel_max * control_time_step;
    if (path.poses.size() == 0)
      return path;

    double dis = 0, ang = 0;
    double margin = _pathResolution * 0.01;
    double remaining = 0;
    int nPoints = 0;

    nav_msgs::Path fixedPath = path;
    fixedPath.poses.clear();
    fixedPath.poses.push_back(path.poses[0]);

    size_t start = 0, next = 1;
    while (next < path.poses.size())
    {
      dis += hypot(path.poses[next].pose.position.x - path.poses[next - 1].pose.position.x,
                   path.poses[next].pose.position.y - path.poses[next - 1].pose.position.y) +
             remaining;
      ang = atan2(path.poses[next].pose.position.y - path.poses[start].pose.position.y,
                  path.poses[next].pose.position.x - path.poses[start].pose.position.x);

      if (dis < _pathResolution - margin)
      {
        next++;
        remaining = 0;
      }
      else if (dis > (_pathResolution + margin))
      {
        geometry_msgs::PoseStamped point_start = path.poses[start];
        nPoints = dis / _pathResolution;
        for (int j = 0; j < nPoints; j++)
        {
          point_start.pose.position.x = point_start.pose.position.x + _pathResolution * cos(ang);
          point_start.pose.position.y = point_start.pose.position.y + _pathResolution * sin(ang);
          point_start.pose.orientation = tf::createQuaternionMsgFromYaw(ang);
          fixedPath.poses.push_back(point_start);
        }
        remaining = dis - nPoints * _pathResolution;
        start++;
        path.poses[start].pose.position = point_start.pose.position;
        dis = 0;
        next++;
      }
      else
      {
        dis = 0;
        remaining = 0;
        fixedPath.poses.push_back(path.poses[next]);
        next++;
        start = next - 1;
      }
    }

    return fixedPath;
  }

  /* SLAM等里程计的回调函数 */
  void odom_cbk(const nav_msgs::OdometryConstPtr& odom_in)
  {
    {
      std::lock_guard<std::mutex> lck(state_mtx);
      cur_odom = *odom_in;
    }
    pubTF();
    odom_available = true;
  }

  inline geometry_msgs::PoseStamped pos2PoseStamped(Eigen::Vector3f pos)
  {
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "odom";
    p.pose.position.x = pos(0);
    p.pose.position.y = pos(1);
    p.pose.position.z = pos(2);
    return p;
  }

  /* 发布机器人到local ENU的TF变换 */
  inline void pubTF()
  {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(
        tf::Vector3(cur_odom.pose.pose.position.x, cur_odom.pose.pose.position.y, cur_odom.pose.pose.position.z));
    q.setW(cur_odom.pose.pose.orientation.w);
    q.setX(cur_odom.pose.pose.orientation.x);
    q.setY(cur_odom.pose.pose.orientation.y);
    q.setZ(cur_odom.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, cur_odom.header.stamp, "odom", "base_link"));
  }

  /* 可视化2D Nav Goal标记的路径点 */
  void visDiscretTrajPoints()
  {
    if (path_point_pub_.getNumSubscribers() == 0)
      return;
    mk_history_points.header.frame_id = "odom";
    mk_history_points.header.stamp = ros::Time::now();
    mk_history_points.type = visualization_msgs::Marker::SPHERE_LIST;
    mk_history_points.action = visualization_msgs::Marker::DELETE;
    mk_history_points.id = 2;

    path_point_pub_.publish(mk_history_points);

    mk_history_points.action = visualization_msgs::Marker::ADD;
    mk_history_points.pose.orientation.x = 0.0;
    mk_history_points.pose.orientation.y = 0.0;
    mk_history_points.pose.orientation.z = 0.0;
    mk_history_points.pose.orientation.w = 1.0;

    mk_history_points.color.r = 1;
    mk_history_points.color.g = 0;
    mk_history_points.color.b = 0;
    mk_history_points.color.a = 1;

    mk_history_points.scale.x = 0.5;
    mk_history_points.scale.y = 0.5;
    mk_history_points.scale.z = 0.5;
    mk_history_points.points.clear();
    mk_history_points.points.reserve(path_tracking_vec.size());
    for (size_t i = 0; i < path_tracking_vec.size(); i++)
    {
      geometry_msgs::Point pt;
      pt.x = path_tracking_vec[i](0);
      pt.y = path_tracking_vec[i](1);
      pt.z = -0.1;
      mk_history_points.points.push_back(pt);
    }

    path_point_pub_.publish(mk_history_points);
    ros::Duration(0.001).sleep();
  }
};

#endif  
