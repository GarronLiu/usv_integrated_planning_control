// Main Contributor:Garron Liu
// Date: 18th Dec. 2023
// Great thanks to the authors of Fast-planner, hku-mars lab for opening their source codes. Without their
// contributions, this code project would not be created successfully.
// Personally thanks for their subtle knowledge in robotic research field, which inspires me a lot!
#ifndef USV_IPC_MANAGER_H
#define USV_IPC_MANAGER_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointPull.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Eigen>
#include <GeographicLib/LocalCartesian.hpp>

#include <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>

#include <bspline/non_uniform_bspline.h>
#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/kinodynamic_astar.h>

using namespace std;
using namespace Eigen;

struct LocalTrajData
{
  /* info of generated traj */
  int traj_id_;
  double duration_;
  double start_time_;
  Eigen::Vector3d start_pos_;
  NonUniformBspline position_traj_, velocity_traj_, acceleration_traj_;
  vector<Eigen::Vector3d> kino_path_;
};
struct OdomState
{
  Eigen::Quaterniond q;
  Eigen::Vector3d v;
  Eigen::Vector3d p;
  Eigen::Vector3d angular_velo;
  double stamp;
};

class UsvIpcManager
{
public:
  ros::NodeHandle node_;
  ros::Subscriber odom_filtered_sub_;
  ros::Subscriber manual_goal_sub_;
  ros::Subscriber fcu_ekf_orig_sub_;
  ros::Subscriber gcs_waypoints_sub_;
  ros::Subscriber state_sub_;
  ros::Publisher traj_horizon_pub_;
  ros::Publisher cmd_fcu_pub_, rc_override_pub_;
  ros::Publisher pubControlPoints;
  ros::Publisher pubPathSearched;
  ros::Publisher pubPathOptimized;
  ros::Publisher pubCollisionPoint;
  ros::Timer control_timer_;
  ros::Timer exec_statetimer_;
  ros::Timer check_collision_timer_;

  ros::ServiceClient mission_pull_client_;

  /* Construct and Initialize Parameters */
  UsvIpcManager()
  {
    odom_filtered_sub_ = node_.subscribe<nav_msgs::Odometry>("/odometry", 1, &UsvIpcManager::odom_cbk, this,
                                                             ros::TransportHints().tcpNoDelay());
    fcu_ekf_orig_sub_ = node_.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/ekf_origin", 1,
                                                                          &UsvIpcManager::setLocalENUOriginLLA, this);

    // Path Planning and Trajectory Optimization
    node_.param<double>("manager/max_vel", max_vel, 1.0);
    node_.param<double>("manager/max_acc", max_acc, 1.0);
    node_.param<double>("manager/max_rot_velocity", max_rot, 1.0);
    node_.param<double>("manager/control_points_distance", ctrl_pt_dist, 1.0);
    node_.param<double>("manager/emergency_distance", emergency_dist_, 3.0);
    node_.param<double>("manager/thresh_no_replan", no_replan_thresh_, 2.0);
    node_.param<double>("manager/thresh_replan", replan_thresh_, 2.0);
    node_.param<double>("manager/goal_tolerance", goal_tolerance_, 2.0);
    node_.param<double>("manager/collision_check_radius", collision_check_radius_, 15.0);
    node_.param<double>("manager/map_resolution", map_resolution_, 0.2);
    node_.param<bool>("manager/pubTF", if_pubTF, true);
    double max_trans_velocity;
    node_.param<double>("opt_tracker/max_trans_velocity", max_trans_velocity, max_vel);
    time_ratio = max_trans_velocity / max_vel;
    state_sub_ = node_.subscribe<mavros_msgs::State>("mavros/state", 5, &UsvIpcManager::state_cb, this);
    gcs_waypoints_sub_ =
        node_.subscribe<mavros_msgs::WaypointList>("/mavros/mission/waypoints", 1, &UsvIpcManager::waypointCb, this);
    mission_pull_client_ = node_.serviceClient<mavros_msgs::WaypointPull>("/mavros/mission/pull");
    manual_goal_sub_ = node_.subscribe<geometry_msgs::PoseStamped>("manager/goal", 1, &UsvIpcManager::goalCb, this);
    exec_statetimer_ = node_.createTimer(ros::Duration(0.05), &UsvIpcManager::execFSMCb, this);
    pubPathSearched = node_.advertise<visualization_msgs::Marker>("manager/trajectory/kino", 10);
    pubPathOptimized = node_.advertise<nav_msgs::Path>("manager/trajectory/tracking", 10);
    pubControlPoints = node_.advertise<visualization_msgs::MarkerArray>("manager/control_points", 10);
    pubCollisionPoint = node_.advertise<geometry_msgs::Point>("/collision_point", 10);
    check_collision_timer_ = node_.createTimer(ros::Duration(0.05), &UsvIpcManager::checkCollisionCb, this);
    vc_map_.reset(new VehicleCentriMap(node_));
    path_searcher_.reset(new KinodynamicAstar);
    path_searcher_->setParam(node_);
    path_searcher_->setEnvironment(vc_map_);
    path_searcher_->init();
    bspline_optimizer_.reset(new BsplineOptimizer);
    bspline_optimizer_->setParam(node_);
    bspline_optimizer_->setEnvironment(vc_map_);

    has_odom_ = false;
    fcu_ekf_origin_received = false;
    target_flag = false;
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /* ==State Estimation== */
  mutex odom_mtx, status_mtx, collision_mtx, waypoint_mtx, goal_mtx, fsm_mtx, control_mtx;
  GeographicLib::LocalCartesian geo_converter;
  OdomState cur_state;
  bool if_pubTF;

  bool has_odom_;
  bool fcu_ekf_origin_received;

  string odometry_topic;

  /* ==Collision-free  Trajectory Generation== */
  geometry_msgs::PoseStamped cur_goal;
  std::queue<Eigen::Vector3d> waypoints_enu_list;
  visualization_msgs::Marker mk_bspline_ctrl_pts;
  vector<Eigen::Vector3d> path_searched_vec;
  vector<Eigen::Vector3d> path_optimized_vec;
  VehicleCentriMap::Ptr vc_map_;
  KinodynamicAstar::Ptr path_searcher_;
  BsplineOptimizer::Ptr bspline_optimizer_;
  double max_vel, max_acc, max_rot, t_span;
  double ctrl_pt_dist;
  bool target_flag;
  LocalTrajData local_traj_data;
  int emergency_hold_cmd_count;
  double emergency_dist_;
  double no_replan_thresh_, replan_thresh_, goal_tolerance_, collision_check_radius_, map_resolution_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_;  // start state
  Eigen::Vector3d end_pt_, end_vel_;                  // target state
  double target_yaw_;
  /* ==Controller Related== */
  double time_ratio;
  double t_start, dt_full;
  Eigen::Vector3d fcu_ekf_origin_offset;

  /* ==FSM== */
  enum FSM_EXEC_STATE
  {
    INIT = 0,
    WAIT_TARGET,
    GEN_NEW_TRAJ,
    REPLAN_TRAJ,
    TRACK_TRAJ,
    EMERGENCY,
    YAW_ALIGNMENT
  };
  FSM_EXEC_STATE exec_state = INIT;

  mavros_msgs::State current_fcu_status;
  const std::string GUIDED_MODE = "GUIDED";
  void state_cb(const mavros_msgs::State::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lck(status_mtx);
    // if FCU mode switch from others to guided, call ros service to pull waypoints list.
    if (current_fcu_status.mode != GUIDED_MODE && msg->mode == GUIDED_MODE)
    {
      mavros_msgs::WaypointPull srv;
      if (mission_pull_client_.call(srv))
      {
        ROS_INFO("Mission pulled successfully!");
      }
    }
    // store current fcu status
    if (current_fcu_status.mode != msg->mode)
    {
      current_fcu_status = *msg;
    }

    if (current_fcu_status.mode != GUIDED_MODE)
    {
      if (exec_state != WAIT_TARGET)
      {
        target_flag = false;
        changeFSMExecState(WAIT_TARGET, "GCS");
      }
      nav_msgs::Path empty_path;
      empty_path.header.frame_id = "odom";
      pubPathOptimized.publish(empty_path);
    }
  }

  /* 接收来自FCU的EKF原点 */
  void setLocalENUOriginLLA(const sensor_msgs::NavSatFixConstPtr& msg)
  {
    if (fcu_ekf_origin_received)
      return;

    geo_converter.Reset(msg->latitude, msg->longitude, msg->altitude);
    fcu_ekf_origin_received = true;

    ROS_INFO("\033[1;36mReceived FCU EKF Origin:[%0.3f %0.3f %0.3f].\033[0m", msg->latitude, msg->longitude,
             msg->altitude);
  }

  void waypointCb(const mavros_msgs::WaypointListConstPtr& msg)
  {
    std::lock_guard<std::mutex> lck(waypoint_mtx);
    if (!fcu_ekf_origin_received)
    {
      ROS_WARN("Local ENU has not been set! Please wait until it's ready and reset mission waypoints in GCS!");
      return;
    }

    while (!waypoints_enu_list.empty())
    {
      waypoints_enu_list.pop();
    }

    for (size_t i = 1; i < msg->waypoints.size(); i++)
    {
      Eigen::Vector3d wp_enu;
      geo_converter.Forward(msg->waypoints[i].x_lat, msg->waypoints[i].y_long, 0, wp_enu.x(), wp_enu.y(), wp_enu.z());
      waypoints_enu_list.push(wp_enu);
      cout << "received waypoint:" << wp_enu.transpose() << endl;
    }

    end_pt_ << waypoints_enu_list.front().x(), waypoints_enu_list.front().y(), 0;
    waypoints_enu_list.pop();
    target_flag = true;
    end_vel_.setZero();

    if (exec_state == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "GCS");
    else if (exec_state == TRACK_TRAJ)
      changeFSMExecState(REPLAN_TRAJ, "GCS");
  }

  void goalCb(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    std::lock_guard<std::mutex> lck(goal_mtx);
    end_pt_ << msg->pose.position.x, msg->pose.position.y, 0;
    target_flag = true;
    end_vel_.setZero();

    if (exec_state == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    else if (exec_state == TRACK_TRAJ)
      changeFSMExecState(REPLAN_TRAJ, "TRIG");
  }

  inline geometry_msgs::PoseStamped pos2PoseStamped(Eigen::Vector3d pos)
  {
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "odom";
    p.pose.position.x = pos(0);
    p.pose.position.y = pos(1);
    p.pose.position.z = pos(2);
    return p;
  }

  /* SLAM等里程计的回调函数 */
  void odom_cbk(const nav_msgs::OdometryConstPtr& odom_in)
  {
    std::lock_guard<std::mutex> lck(odom_mtx);
    cur_state.p =
        Eigen::Vector3d(odom_in->pose.pose.position.x, odom_in->pose.pose.position.y, odom_in->pose.pose.position.z);
    cur_state.v =
        Eigen::Vector3d(odom_in->twist.twist.linear.x, odom_in->twist.twist.linear.y, odom_in->twist.twist.linear.z);
    cur_state.angular_velo =
        Eigen::Vector3d(odom_in->twist.twist.angular.x, odom_in->twist.twist.angular.y, odom_in->twist.twist.angular.z);
    cur_state.q.w() = odom_in->pose.pose.orientation.w;
    cur_state.q.x() = odom_in->pose.pose.orientation.x;
    cur_state.q.y() = odom_in->pose.pose.orientation.y;
    cur_state.q.z() = odom_in->pose.pose.orientation.z;
    cur_state.stamp = odom_in->header.stamp.toSec();

    if (if_pubTF)
    {
      pubTF(*odom_in);
    }
    has_odom_ = true;
  }

  /* 发布机器人到local ENU的TF变换 */
  inline void pubTF(const nav_msgs::Odometry& _state)
  {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(
        tf::Vector3(_state.pose.pose.position.x, _state.pose.pose.position.y, _state.pose.pose.position.z));
    q.setW(_state.pose.pose.orientation.w);
    q.setX(_state.pose.pose.orientation.x);
    q.setY(_state.pose.pose.orientation.y);
    q.setZ(_state.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, _state.header.stamp, "odom", "base_link"));
  }

  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {
    string state_str[7] = { "INIT",       "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ",
                            "TRACK_TRAJ", "EMERGENCY",   "YAW_ALIGNMENT" };
    int pre_s = int(exec_state);
    exec_state = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void printFSMExecState()
  {
    string state_str[7] = { "INIT",       "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ",
                            "TRACK_TRAJ", "EMERGENCY",   "YAW_ALIGNMENT" };

    cout << "[FSM]: state: " + state_str[int(exec_state)] << endl;
  }

  void publishTargetPoint(Eigen::Vector2d pos, double yaw)
  {
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "odom";
    p.pose.position.x = pos(0);
    p.pose.position.y = pos(1);
    p.pose.position.z = 0.05;
    p.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    nav_msgs::Path path;
    path.header.frame_id = "odom";
    path.header.stamp = ros::Time::now();
    path.poses.push_back(p);
    pubPathOptimized.publish(path);
  }

  void execFSMCb(const ros::TimerEvent&)
  {
    std::lock_guard<std::mutex> lck(fsm_mtx);

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!has_odom_)
        cout << "no odom." << endl;
      if (!target_flag)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state)
    {
      case INIT: {
        if (!has_odom_)
        {
          return;
        }
        changeFSMExecState(WAIT_TARGET, "FSM");
        Eigen::Vector2d targetPos(cur_state.p(0), cur_state.p(1));
        geometry_msgs::Quaternion q;
        q.w = cur_state.q.w();
        q.x = cur_state.q.x();
        q.y = cur_state.q.y();
        q.z = cur_state.q.z();
        double targetYaw = tf::getYaw(q);
        publishTargetPoint(targetPos, targetYaw);
        break;
      }

      case WAIT_TARGET: {
        if (!target_flag)
        {
          return;
        }
        else
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        break;
      }

      case GEN_NEW_TRAJ: {
        double cur_yaw_;
        start_pt_ = Eigen::Vector3d(cur_state.p(0), cur_state.p(1), 0.0);
        // start_vel_ = cur_state.q*Eigen::Vector3d(max_vel,0.0,0.0);
        start_vel_ = cur_state.q * Eigen::Vector3d(cur_state.v(0), cur_state.v(1), 0.0);
        Eigen::Vector3d cur_rot_vector =
            cur_state.q.toRotationMatrix().block(0, 0, 3, 1);  //旋转矩阵元素rot_1_1=cos(Pitch)*cos(Yaw)
        cur_yaw_ = atan2(cur_rot_vector(1), cur_rot_vector(0));
        // start_acc_= cur_state.q*Eigen::Vector3d(0.5*max_acc,0.0,0.0);
        start_acc_ = Eigen::Vector3d(0.0, 0.0, 0.0);

        bool success = callTrajectoryReplan();

        if (success)
        {
          //无人船需要考虑当前艏向与轨迹朝向是否大致对齐
          Eigen::Vector3d traj_vel = local_traj_data.position_traj_.evaluateDeBoorT(0.1)-start_pt_;
          // Eigen::Vector3d traj_vel = end_pt_ - start_pt_;
          target_yaw_ = atan2(traj_vel(1), traj_vel(0));
          if (!checkYawAlignment(cur_yaw_, target_yaw_))
          {
            Eigen::Vector2d targetPos(cur_state.p(0), cur_state.p(1));
            publishTargetPoint(targetPos, target_yaw_);
            changeFSMExecState(YAW_ALIGNMENT, "FSM");
            break;  //退出当前switch
          }
          nav_msgs::Path traj_track_full;
          discretRefTraj(traj_track_full);
          pubPathOptimized.publish(traj_track_full);
          changeFSMExecState(TRACK_TRAJ, "FSM");
        }
        else
        {
          Eigen::Vector2d targetPos(cur_state.p(0), cur_state.p(1));
          geometry_msgs::Quaternion q;
          q.w = cur_state.q.w();
          q.x = cur_state.q.x();
          q.y = cur_state.q.y();
          q.z = cur_state.q.z();
          double targetYaw = tf::getYaw(q);
          publishTargetPoint(targetPos, targetYaw);
          if ((start_pt_ - end_pt_).norm() < goal_tolerance_)
          {
            target_flag = false;
            changeFSMExecState(WAIT_TARGET, "FSM");
          }
          else
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          }
        }
        break;
      }

      case REPLAN_TRAJ: {
        double time_now = ros::Time::now().toSec();
        double t_cur = time_now - local_traj_data.start_time_;
        start_pt_ = local_traj_data.position_traj_.evaluateDeBoorT(t_cur * time_ratio);

        start_pt_ = Eigen::Vector3d(cur_state.p(0), cur_state.p(1), 0.0);
        // start_vel_ = cur_state.q*Eigen::Vector3d(max_vel,0.0,0.0);
        start_vel_ = cur_state.q * Eigen::Vector3d(cur_state.v(0), cur_state.v(1), 0.0);
        // start_acc_= cur_state.q*Eigen::Vector3d(0.1*max_acc,0.0,0.0);
        start_acc_ = Eigen::Vector3d(0.0, 0.0, 0.0);

        bool success = callTrajectoryReplan();

        if (success)
        {
          nav_msgs::Path traj_track_full;
          discretRefTraj(traj_track_full);
          pubPathOptimized.publish(traj_track_full);
          changeFSMExecState(TRACK_TRAJ, "FSM");
        }
        else
        {
          Eigen::Vector2d targetPos(cur_state.p(0), cur_state.p(1));
          geometry_msgs::Quaternion q;
          q.w = cur_state.q.w();
          q.x = cur_state.q.x();
          q.y = cur_state.q.y();
          q.z = cur_state.q.z();
          double targetYaw = tf::getYaw(q);
          publishTargetPoint(targetPos, targetYaw);
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }

        break;
      }

      case TRACK_TRAJ: {
        /* determine if need to replan */
        ros::Time time_now = ros::Time::now();
        double t_cur = time_now.toSec() - local_traj_data.start_time_;
        t_cur = min(local_traj_data.duration_, t_cur * time_ratio);

        // Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);
        Eigen::Vector3d pos;
        pos = cur_state.p;
        pos(2) = 0;
        
        // if (t_cur > local_traj_data.duration_ - 1e-2 && (end_pt_ - pos).norm() < goal_tolerance_) {
        if ((end_pt_ - pos).norm() < goal_tolerance_)
        {
          if (waypoints_enu_list.empty())
          {
            target_flag = false;
            changeFSMExecState(WAIT_TARGET, "FSM");
            Eigen::Vector2d targetPos(end_pt_(0), end_pt_(1));
            geometry_msgs::Quaternion q;
            q.w = cur_state.q.w();
            q.x = cur_state.q.x();
            q.y = cur_state.q.y();
            q.z = cur_state.q.z();
            double targetYaw = tf::getYaw(q);
            publishTargetPoint(targetPos, targetYaw);
            return;
          }
          else
          {
            end_pt_ << waypoints_enu_list.front().x(), waypoints_enu_list.front().y(), 0;
            waypoints_enu_list.pop();
            target_flag = true;
            end_vel_.setZero();
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            ROS_INFO("Heading to next mission waypoint called by GCS!");
          }
        }
        else if ((end_pt_ - pos).norm() < no_replan_thresh_)
        {
          // cout << "near end" << endl;
          return;
        }
        else if ((local_traj_data.start_pos_ - pos).norm() < replan_thresh_)
        {
          // cout << "near start" << endl;
          return;
        }
        else
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
        break;
      }

      case EMERGENCY: {
        if (emergency_hold_cmd_count < 20)
        {
          emergency_hold_cmd_count++;
        }
        else
        {
          // nav_msgs::Path empty_path;
          // empty_path.header.frame_id = "odom";
          // pubPathOptimized.publish(empty_path);
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        break;
      }

      case YAW_ALIGNMENT: {
        Eigen::Vector3d cur_rot_vector =
            cur_state.q.toRotationMatrix().block(0, 0, 3, 1);  //旋转矩阵元素rot_1_1=cos(Pitch)*cos(Yaw)
        double cur_yaw_ = atan2(cur_rot_vector(1), cur_rot_vector(0));
        if (checkYawAlignment(cur_yaw_, target_yaw_))
        {
          nav_msgs::Path traj_track_full;
          discretRefTraj(traj_track_full);
          pubPathOptimized.publish(traj_track_full);
          changeFSMExecState(TRACK_TRAJ, "FSM");
        }
        break;
      }
    }
  }

  bool callTrajectoryReplan(void)
  {
    std::cout << "[Replan]: -----------------------" << std::endl;
    cout << "start: " << start_pt_.transpose() << ", " << start_vel_.transpose() << ", " << start_acc_.transpose()
         << "\ngoal:" << end_pt_.transpose() << ", " << end_vel_.transpose() << endl;

    if ((start_pt_ - end_pt_).norm() < 0.2)
    {
      cout << "Close goal" << endl;
      return false;
    }  //目标点与起始点距离小于0.2m就没必要规划了

    ros::Time t1, t2;
    double t_search = 0.0, t_opt = 0.0, t_adjust = 0.0;

    t1 = ros::Time::now();

    // Path searching
    path_searcher_->reset();
    int status = path_searcher_->search(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, true);
    if (status == KinodynamicAstar::NO_PATH)
    {
      std::cout << "[Replan]: kinodynamic searching failed" << endl;
      // retry searching with discontinuous initial state
      path_searcher_->reset();
      status = path_searcher_->search(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, false);

      if (status == KinodynamicAstar::NO_PATH)
      {
        ROS_INFO("[Replan]: Can't find path");
        return false;
      }
      else
      {
        ROS_INFO("[Replan]: retry search success.");
      }
    }
    else
    {
      ROS_INFO("[Replan]: kinodynamic search success.");
    }

    local_traj_data.kino_path_ = path_searcher_->getKinoTraj(0.05);

    t_search = (ros::Time::now() - t1).toSec();
    // Trajectory optimization

    // 1.calculate B-spline control points for A* path
    double ts = 2.0 * ctrl_pt_dist / max_vel;
    std::vector<Eigen::Vector3d> point_set, start_end_derivatives;
    path_searcher_->getSamples(ts, point_set, start_end_derivatives);
    Eigen::MatrixXd ctrl_pts;
    NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    t1 = ros::Time::now();

    // 2.B-spline trajectory optimization
    /// TODO: add constraint of control points to gcs cruise path(让轨迹更加靠近任务航点构成的直线) or guide path to the
    /// center line of water way(飞行走廊膨胀算法来生成指引)

    int cost_function = BsplineOptimizer::NORMAL_PHASE;
    if (status == KinodynamicAstar::REACH_END)
    {
      cost_function |= BsplineOptimizer::ENDPOINT;
    }
    bspline_optimizer_->setEnvironment(vc_map_);
    ctrl_pts = bspline_optimizer_->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);

    t_opt = (ros::Time::now() - t1).toSec();

    t1 = ros::Time::now();
    local_traj_data.position_traj_ = NonUniformBspline(ctrl_pts, 3, ts);
    double to = local_traj_data.position_traj_.getTimeSum();
    local_traj_data.position_traj_.setPhysicalLimits(max_vel, max_acc);
    bool feasible = local_traj_data.position_traj_.checkFeasibility(false);

    int iter_num = 0;
    while (!feasible && ros::ok())
    {
      feasible = local_traj_data.position_traj_.reallocateTime();
      if (++iter_num >= 3)
        break;
    }
    double tn = local_traj_data.position_traj_.getTimeSum();
#ifdef DEBUG
    cout << "[Replan]: Reallocate ratio: " << tn / to << endl;
#endif
    if (tn / to > 3.0)
      ROS_ERROR("reallocate error.");

    t_adjust = (ros::Time::now() - t1).toSec();

    double t_total = t_search + t_opt + t_adjust;

#ifdef DEBUG
    cout << "[kino replan]: time: " << t_total << ", search: " << t_search << ", optimize: " << t_opt
         << ", adjust time:" << t_adjust << endl;
#endif

    updateTrajInfo();

    visualizeControlPts(ctrl_pts);

    visKinoTraj();

    return true;
  }

  void updateTrajInfo()
  {
    local_traj_data.start_time_ = ros::Time::now().toSec();
    local_traj_data.velocity_traj_ = local_traj_data.position_traj_.getDerivative();
    local_traj_data.acceleration_traj_ = local_traj_data.velocity_traj_.getDerivative();
    local_traj_data.start_pos_ = local_traj_data.position_traj_.evaluateDeBoorT(0.0);
    local_traj_data.duration_ = local_traj_data.position_traj_.getTimeSum();
    local_traj_data.traj_id_ += 1;
  }

  void visualizeControlPts(const Eigen::MatrixXd& pts)
  {
    visualization_msgs::MarkerArray mk_ary;

    visualization_msgs::Marker mk;
    mk.header.frame_id = "odom";
    mk.header.stamp = ros::Time::now();
    mk.ns = "control_points";

    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.color.r = 1.0;
    mk.color.b = 0.0;
    mk.color.g = 0.0;
    mk.color.a = 1.0;
    mk.scale.x = 0.5;
    mk.scale.y = 0.5;
    mk.scale.z = 0.5;

    for (size_t i = 0; i < pts.rows(); i++)
    {
      mk.pose.position.x = pts(i, 0);
      mk.pose.position.y = pts(i, 1);
      mk.pose.position.z = pts(i, 2);
      mk.id = i;
      mk_ary.markers.push_back(mk);
    }
    pubControlPoints.publish(mk_ary);
  }

  void visKinoTraj()
  {
    if (pubPathSearched.getNumSubscribers() == 0)
      return;
    visualization_msgs::Marker mk;
    mk.header.frame_id = "odom";
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.action = visualization_msgs::Marker::DELETE;
    mk.id = 2;

    pubPathSearched.publish(mk);

    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = 1;
    mk.color.g = 1;
    mk.color.b = 0;
    mk.color.a = 0.4;

    mk.scale.x = 0.075;
    mk.scale.y = 0.075;
    mk.scale.z = 0.075;
    mk.points.clear();
    mk.points.reserve(local_traj_data.kino_path_.size());
    for (size_t i = 0; i < local_traj_data.kino_path_.size(); i++)
    {
      geometry_msgs::Point pt;
      pt.x = local_traj_data.kino_path_[i](0);
      pt.y = local_traj_data.kino_path_[i](1);
      pt.z = -0.1;
      mk.points.push_back(pt);
    }

    pubPathSearched.publish(mk);
  }

  void checkCollisionCb(const ros::TimerEvent& e)
  {
    std::lock_guard<std::mutex> lck(collision_mtx);
    if (target_flag)
    {
      double dist = vc_map_->evaluateCoarseEDT(Eigen::Vector2d(end_pt_(0), end_pt_(1)));

      if (dist <= 0.3)
      {
        bool new_goal = false;
        const double dr = 0.5, dtheta = 30;
        double new_x, new_y, max_dist = -1.0;
        Eigen::Vector2d goal;

        for (double r = dr; r <= 5 * dr + 1e-3; r += dr)
        {
          for (double theta = -90; theta <= 270; theta += dtheta)
          {
            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);

            Eigen::Vector2d new_pt(new_x, new_y);
            dist = vc_map_->evaluateCoarseEDT(new_pt);

            if (dist > max_dist)
            {
              /* reset end_pt_ */
              goal(0) = new_x;
              goal(1) = new_y;
              max_dist = dist;
            }
          }
        }

        if (max_dist > 0.3)
        {
          cout << "change goal, replan." << endl;
          end_pt_ << goal(0), goal(1), 0;
          target_flag = true;
          end_vel_.setZero();

          if (exec_state == TRACK_TRAJ)
          {
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
        }
        else
        {
          // have_target_ = false;
          // cout << "Goal near collision, stop." << endl;
          // changeFSMExecState(WAIT_TARGET, "SAFETY");
          cout << "goal near collision, keep retry" << endl;
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }

      /* ---------- check trajectory ---------- */
      if (exec_state == FSM_EXEC_STATE::TRACK_TRAJ)
      {
        double dist;
        bool safe = checkTrajCollision(dist);

        if (!safe)
        {
          // cout << "current traj in collision." << endl;
          if (dist < emergency_dist_)
          {
            ROS_WARN("Emergency confront! Slow Down!");
            changeFSMExecState(EMERGENCY, "SAFETY");
            Eigen::Vector2d targetPos(cur_state.p(0), cur_state.p(1));
            geometry_msgs::Quaternion q;
            q.w = cur_state.q.w();
            q.x = cur_state.q.x();
            q.y = cur_state.q.y();
            q.z = cur_state.q.z();
            double targetYaw = tf::getYaw(q);
            publishTargetPoint(targetPos, targetYaw);
            emergency_hold_cmd_count = 0;
          }
          else
          {
            ROS_WARN("current traj in collision.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
        }
      }
    }
  }

  bool checkTrajCollision(double& distance)
  {
    double t_now = (ros::Time::now().toSec() - local_traj_data.start_time_) * time_ratio;

    double tm, tmp;
    local_traj_data.position_traj_.getTimeSpan(tm, tmp);
    Eigen::Vector3d cur_pt = local_traj_data.position_traj_.evaluateDeBoor(tm + t_now);

    double radius = 0.0;
    // cout << "current traj in collision." << endl;
    Eigen::Vector3d fut_pt;
    static double check_res = map_resolution_ / max_vel;
    double fut_t = check_res;  //(map_resolution/max_vel)

    while (radius < collision_check_radius_ && t_now + fut_t < local_traj_data.duration_)
    {
      fut_pt = local_traj_data.position_traj_.evaluateDeBoor(tm + t_now + fut_t);

      double dist = vc_map_->evaluateCoarseEDT(Eigen::Vector2d(fut_pt(0), fut_pt(1)));
      if (dist < 0.1)
      {
        distance = radius;
        geometry_msgs::Point collision_point;
        collision_point.x = fut_pt(0);
        collision_point.y = fut_pt(1);
        pubCollisionPoint.publish(collision_point);
        return false;
      }

      radius = (fut_pt - cur_pt).norm();
      fut_t += check_res;
    }

    return true;
  }

  bool checkYawAlignment(double cur_yaw, double target_yaw)
  {
    double d_yaw = cur_yaw - target_yaw;
    while (d_yaw < -M_PI)
    {
      d_yaw += 2 * M_PI;
    }
    while (d_yaw > M_PI)
    {
      d_yaw -= 2 * M_PI;
    }

    static double yaw_thres = M_PI / 6;
    if (d_yaw > yaw_thres)
    {
      return false;
    }
    else if (d_yaw < -yaw_thres)
    {
      return false;
    }
    return true;
  }

  void discretRefTraj(nav_msgs::Path& path)
  {
    dt_full = local_traj_data.duration_;

    path.header.frame_id = "odom";
    path.header.stamp = ros::Time::now();

    double res = 0.05;
    path.poses.reserve(ceil(dt_full / res));

    for (double dt = res; dt <= dt_full; dt += res)
    {
      Eigen::Vector3d ref_pos = local_traj_data.position_traj_.evaluateDeBoorT(dt);
      path.poses.push_back(pos2PoseStamped(Eigen::Vector3d(ref_pos(0), ref_pos(1), -0.05)));
    }

    local_traj_data.start_time_ = ros::Time::now().toSec();
  }
};

#endif
