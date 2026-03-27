
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>

#include <std_msgs/Float64MultiArray.h>

#include <usv_opt_tracker/usv_opt_tracker.h>

#include <iostream>
#include <mutex>
#include <condition_variable>
#include <Eigen/Eigen>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <thread>

#include <dynamic_reconfigure/server.h>
#include <usv_opt_tracker/tunesConfig.h>

class ControlInterface
{
public:
  ros::NodeHandle node_;
  ros::Subscriber reference_traj_sub_;
  ros::Subscriber odom_sub_;
  ros::Timer control_timer_;
  ros::Publisher cmd_fcu_pub_;
  ros::Publisher rc_override_pub_;
  ros::Publisher traj_horizon_pub_;
  ros::Publisher traj_predict_pub_;
  ros::Publisher acc_error_pub_;
  ros::Publisher track_error_pub_;
  ros::Publisher solution_time_pub_;
  dynamic_reconfigure::Server<dynamic_tunes::tunesConfig> server_;
  dynamic_reconfigure::Server<dynamic_tunes::tunesConfig>::CallbackType f_;

  double control_frequency;
  double control_delay;
  double nmpc_pred_hori;
  double time_interval;
  int nmpc_pred_hori_num;
  int cmd_type;
  bool gpr_update_enable;  // param for whether to update GP online
  bool gpr_enable;         // param for whether to use GPR augmented model
  nav_msgs::Path current_ref_traj;
  nav_msgs::Odometry current_odom;
  bool have_odom;
  double traj_ref_vel;
  size_t traj_start_id;
  std::unique_ptr<usv_system::UsvOptTracker> opt_tracker_;

  ControlInterface(ros::NodeHandle& nh) : node_(nh)
  {
    node_.param<double>("ControlFrequency", control_frequency, 20.0);
    node_.param<double>("ControlDelay", control_delay, 0.1);
    node_.param<int>("CommandType", cmd_type, 0);
    node_.param<bool>("GPUpateEnable", gpr_update_enable, false);
    node_.param<bool>("GPREnable", gpr_enable, false);
    reference_traj_sub_ = node_.subscribe<nav_msgs::Path>("/trajectory", 1, &ControlInterface::trajCbk, this,
                                                          ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom", 1, &ControlInterface::odomCbk, this,
                                                 ros::TransportHints().tcpNoDelay());
    opt_tracker_.reset(new usv_system::UsvOptTracker(gpr_enable));

    // control_timer_ = node_.createTimer(ros::Duration(1/control_frequency),&ControlInterface::loopController,this);
    if (cmd_type == 0)
    {
      cmd_fcu_pub_ = node_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    }
    else
    {
      rc_override_pub_ = node_.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
    }
    traj_horizon_pub_ = node_.advertise<nav_msgs::Path>("tracker/trajectory/horizon", 2);
    traj_predict_pub_ = node_.advertise<nav_msgs::Path>("tracker/trajectory/nmpc_predict", 2);
    acc_error_pub_ = node_.advertise<std_msgs::Float64MultiArray>("tracker/acceleration_error", 2);
    solution_time_pub_ = node_.advertise<std_msgs::Float64MultiArray>("tracker/solution_time", 2);
    track_error_pub_ = node_.advertise<std_msgs::Float64MultiArray>("tracker/track_error", 2);

    nmpc_pred_hori = opt_tracker_->getMpcHorizon();
    time_interval = opt_tracker_->getTimeInterval();
    nmpc_pred_hori_num = round(nmpc_pred_hori / time_interval);
    traj_ref_vel = opt_tracker_->getMaxVelocity();
    current_ref_traj = nav_msgs::Path();
    have_odom = false;
    control_state.setZero();

    f_ = boost::bind(&ControlInterface::configCallback, this, _1, _2);
    server_.setCallback(f_);
  }

  void configCallback(dynamic_tunes::tunesConfig& config, uint32_t level)
  {
    traj_ref_vel = config.velocity;
    gpr_update_enable = config.GPUpdate;
  }

  void trajCbk(const nav_msgs::PathConstPtr& msg)
  {
    std::lock_guard<std::mutex> lck(traj_mtx);
    nav_msgs::Path pathRaw = *msg;
    int length = pathRaw.poses.size();
    if (length == 0)
    {
      current_ref_traj = nav_msgs::Path();
      releaseRCOverride();
    }
    else if (length == 1)
    {
      current_ref_traj = pathRaw;
    }
    else
    {
      double vel_max = traj_ref_vel;
      current_ref_traj = processPath(pathRaw, vel_max);
      traj_start_id = 0;
    }
  }

  void odomCbk(const nav_msgs::OdometryConstPtr& odom_in)
  {
    std::lock_guard<std::mutex> lck(odom_mtx);
    current_odom = *odom_in;
    have_odom = true;
  }

  void loopController()
  {
    ros::Rate rate(control_frequency);
    ct::core::StateVector<usv::USVDynamicModel::STATE_DIM> x_state_last;
    ct::core::StateVector<usv::USVDynamicModel::STATE_DIM> x_ref_last;
    ct::core::StateVectorArray<usv::USVDynamicModel::STATE_DIM> x_pred_traj;
    ct::core::ControlVector<usv::USVDynamicModel::CONTROL_DIM> control_opt;

    while (ros::ok())
    {
      std::lock_guard<std::mutex> lck(control_mtx);
      if (!have_odom)
        continue;

      if (current_ref_traj.poses.empty())
        continue;

      auto t1 = ros::Time::now().toNSec();
      if (current_ref_traj.poses.size() > 0)
      {
        std::lock_guard<std::mutex> lck2(traj_mtx);
        // truncate path
        int min_id = -1;
        float min_dist = FLT_MAX;
        int size = current_ref_traj.poses.size();
        Eigen::Vector2f robot_pos;
        robot_pos[0] = static_cast<float>(current_odom.pose.pose.position.x);
        robot_pos[1] = static_cast<float>(current_odom.pose.pose.position.y);
        for (int i = 0; i < size; ++i)
        {
          Eigen::Vector2f p;
          p[0] = static_cast<float>(current_ref_traj.poses[i].pose.position.x);
          p[1] = static_cast<float>(current_ref_traj.poses[i].pose.position.y);

          float dist = (robot_pos - p).squaredNorm();
          if (dist < min_dist)
          {
            min_dist = dist;
            min_id = i;
          }
        }
        min_id = min(size - 2, min_id);
        if (min_id >= 0 && min_dist < 1.0)
          current_ref_traj.poses.erase(current_ref_traj.poses.begin(), current_ref_traj.poses.begin() + min_id);
      }

      //获取当前状态
      ct::core::StateVector<usv::USVDynamicModel::STATE_DIM> x_state_now;
      {
        std::lock_guard<std::mutex> lck1(odom_mtx);
        double cur_yaw = tf::getYaw(current_odom.pose.pose.orientation);
        x_state_now.setZero();
        x_state_now(0) = current_odom.pose.pose.position.x;
        x_state_now(1) = current_odom.pose.pose.position.y;
        x_state_now(4) = cur_yaw;
        x_state_now(2) = cos(x_state_now(4));
        x_state_now(3) = sin(x_state_now(4));
        x_state_now(5) = current_odom.twist.twist.linear.x;
        x_state_now(6) = current_odom.twist.twist.linear.y;
        x_state_now(7) = current_odom.twist.twist.angular.z;
        x_state_now(8) = control_state(0);
        x_state_now(9) = control_state(1);
      }

      //获取当前参考
      ct::core::StateTrajectory<usv::USVDynamicModel::STATE_DIM> x_ref_traj(InterpolationType::LIN);
      ct::core::ControlTrajectory<usv::USVDynamicModel::CONTROL_DIM> u_ref_traj(InterpolationType::LIN);

      Eigen::Vector3d cmd_pos;
      Eigen::Vector3d cmd_vel;
      double cmd_yaw_rate = 0.0;

      nav_msgs::Path traj_hori;
      traj_hori.header.frame_id = "odom";
      traj_hori.header.stamp = ros::Time::now();
      traj_hori.poses.reserve(nmpc_pred_hori_num);
      {
        std::lock_guard<std::mutex> lck2(traj_mtx);

        for (size_t i = 0; i < nmpc_pred_hori_num; i++)
        {
          traj_hori.poses.push_back(current_ref_traj.poses[min(i, current_ref_traj.poses.size() - 1)]);
        }
      }
      traj_horizon_pub_.publish(traj_hori);
      ROS_ASSERT(traj_hori.poses.size() == nmpc_pred_hori_num);

      for (size_t i = 0; i < traj_hori.poses.size(); i++)
      {
        ct::core::StateVector<usv::USVDynamicModel::STATE_DIM> x_ref;
        x_ref.setZero();
        x_ref(0) = traj_hori.poses[i].pose.position.x;
        x_ref(1) = traj_hori.poses[i].pose.position.y;
        double ref_yaw = tf::getYaw(traj_hori.poses[i].pose.orientation);
        x_ref(2) = cos(ref_yaw);  // yaw
        x_ref(3) = sin(ref_yaw);
        x_ref(4) = ref_yaw;

        ct::core::ControlVector<usv::USVDynamicModel::CONTROL_DIM> u_ref;
        u_ref.setZero();

        double t = time_interval * i;
        x_ref_traj.push_back(x_ref, t, true);
        u_ref_traj.push_back(u_ref, t, true);
      }

      //更新nmpc求解器
      opt_tracker_->update_mpc_tracker(x_state_now, x_ref_traj, u_ref_traj);

      //获取当前最优控制
      control_opt = opt_tracker_->getMPCControl(x_state_now, x_pred_traj);

      visPredTraj(x_pred_traj);

      control_state(0) = x_pred_traj[1](8);
      control_state(1) = x_pred_traj[1](9);

      if (cmd_type == 0)
      {
        pubCmdToFCU(x_pred_traj[10](5), x_pred_traj[10](7));
      }
      else
      {
        pubRCOverrideToFCU(control_state(0), control_state(1));
      }
      //记录发布最优控制的时间及当前状态
      auto t2 = ros::Time::now().toNSec();
      std_msgs::Float64MultiArray solution_time_msg;
      solution_time_msg.data.push_back((t2 - t1) / 1e6);
      solution_time_pub_.publish(solution_time_msg);

      std_msgs::Float64MultiArray track_error_msg;
      track_error_msg.data.push_back(x_ref_last(0) - x_state_now(0));
      track_error_msg.data.push_back(x_ref_last(1) - x_state_now(1));
      track_error_pub_.publish(track_error_msg);
      x_state_last = x_state_now;
      x_ref_last = x_ref_traj[0];
      // cout<<"control loop spend"<<t2-t1<<" ns "<<endl;
      rate.sleep();
    }
  }

  void pubCmdToFCU(const double& cmd_surge_speed, const double& cmd_yaw_rate)
  {
    mavros_msgs::PositionTarget cmd_fcu;
    cmd_fcu.header.stamp = ros::Time::now();
    cmd_fcu.header.frame_id = "odom";

    /* For NMPC control: Velocity and Yaw Rate Control Command in local enu */
    cmd_fcu.velocity.x = cmd_surge_speed;
    cmd_fcu.velocity.y = 0.0;
    cmd_fcu.yaw_rate = cmd_yaw_rate;
    cmd_fcu.type_mask = 0b010111100111;  // velocity and yaw rate;
    cmd_fcu.coordinate_frame = 8;

    cmd_fcu_pub_.publish(cmd_fcu);
    // reference
    // https://ardupilot.org/dev/docs/mavlink-rover-commands.html?highlight=guided#mavlink-rover-commands-set-attitude-target
  }

  void pubRCOverrideToFCU(const double& RC1, const double& RC3)
  {
    mavros_msgs::OverrideRCIn rc_fcu;
    rc_fcu.channels.assign(mavros_msgs::OverrideRCIn::CHAN_RELEASE);
    rc_fcu.channels[0] = static_cast<uint16_t>(RC1 * 1000 + 1500);
    rc_fcu.channels[2] = static_cast<uint16_t>(RC3 * 1000 + 1500);
    rc_override_pub_.publish(rc_fcu);
  }

  void releaseRCOverride()
  {
    mavros_msgs::OverrideRCIn rc_fcu;
    rc_fcu.channels.assign(mavros_msgs::OverrideRCIn::CHAN_RELEASE);
    rc_override_pub_.publish(rc_fcu);
    control_state.setZero();
  }

  void visPredTraj(const ct::core::StateVectorArray<usv::USVDynamicModel::STATE_DIM>& traj_predict_vec_)
  {
    if (traj_predict_pub_.getNumSubscribers() == 0)
      return;
    nav_msgs::Path traj;
    traj.header.frame_id = "odom";
    traj.header.stamp = ros::Time::now();
    traj.poses.reserve(traj_predict_vec_.size());

    for (size_t i = 0; i < traj_predict_vec_.size(); i++)
    {
      auto pos_temp = traj_predict_vec_[i];
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "odom";
      p.pose.position.x = pos_temp(0);
      p.pose.position.y = pos_temp(1);
      p.pose.position.z = 0.05;
      traj.poses.push_back(p);
    }
    traj_predict_pub_.publish(traj);
  }

  nav_msgs::Path processPath(nav_msgs::Path pathIn, double vel_max)
  {
    pathIn = calculatePathYaw(pathIn);
    pathIn = fixPathDensity(pathIn, vel_max);
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

  nav_msgs::Path fixPathDensity(nav_msgs::Path path, double vel_max)
  {
    double _pathResolution = vel_max * time_interval;  //这里应该给实际跟踪的最大速度
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

private:
  /* ==State Estimation== */
  std::mutex odom_mtx, traj_mtx, control_mtx;
  Eigen::Matrix<double, usv::USVDynamicModel::CONTROL_DIM, 1> control_state;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "usv_opt_tracker");
  ros::NodeHandle nh;
  ROS_INFO("\033[1;32m----> USV Opt Track Process Started.\033[0m");
  ControlInterface ci(nh);

  std::thread controlThread(&ControlInterface::loopController, &ci);

  ros::spin();

  controlThread.join();

  return 0;
}
