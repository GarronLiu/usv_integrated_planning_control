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
#include <csignal>
#include <eskf_estimator/eskf_estimator.h>
#include <GeographicLib/LocalCartesian.hpp>

#include  <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>

static GeographicLib::LocalCartesian geo_converter;

static queue<eskf::OBSERV_data> pose_enu_buff_;
static queue<sensor_msgs::Imu> imuMsg_buff_;
static eskf::Measure cur_measure;
static sensor_msgs::Imu imu_last;
static eskf::State cur_state;
std::unique_ptr<eskf::eskf> state_estimator;

bool eskf_initialized = false;
bool local_enu_lla_set = false;
bool fcu_ekf_origin_offset_received = false;
//lock线程锁
std::mutex measure_mtx;
std::mutex traj_mtx;
std::mutex state_mtx;

//ros things
ros::Subscriber imu_sub_;
ros::Subscriber gps_sub_;
ros::Subscriber odom_sub_;
ros::Subscriber fcu_ekf_orig_sub_;

ros::Publisher gps_path_pub_;
ros::Publisher filtered_odom_pub_;
nav_msgs::Path gps_trajectory;

inline geometry_msgs::PoseStamped pos2PoseStamped(Eigen::Vector3d pos){
        geometry_msgs::PoseStamped p;
            p.header.frame_id = "odom";
            p.pose.position.x = pos(0);
            p.pose.position.y = pos(1);
            p.pose.position.z = pos(2);
        return p;
}

/* GPS回调函数 */ // 观测值
static void gps_cbk(const sensor_msgs::NavSatFixConstPtr& gps_in)
{   
    std::lock_guard<std::mutex> lck(measure_mtx);     
    if(!fcu_ekf_origin_offset_received) return;

    sensor_msgs::NavSatFix gps_cur(*gps_in);

    if(gps_cur.position_covariance[0] > 1.0 ||gps_cur.position_covariance[4] > 1.0 )
        return;
    
    double x_enu,y_enu,z_enu;
        geo_converter.Forward(gps_cur.latitude,gps_cur.longitude,gps_cur.altitude,x_enu,y_enu,z_enu);

    eskf::OBSERV_data cur_observ;
        cur_observ.p = Eigen::Vector3d(x_enu,y_enu,z_enu);
        cur_observ.stamp = gps_cur.header.stamp.toSec();
    
    if(gps_path_pub_.getNumSubscribers()!=0){
        geometry_msgs::PoseStamped pose_gps = pos2PoseStamped(cur_observ.p);
        pose_gps.header.stamp = gps_cur.header.stamp;
        gps_trajectory.poses.push_back(pose_gps);
        gps_path_pub_.publish(gps_trajectory);
    }
    pose_enu_buff_.push(cur_observ);
}

/* SLAM等里程计的回调函数 */
void odom_cbk(const nav_msgs::OdometryConstPtr& odom_in)
{    
    std::lock_guard<std::mutex> lck(measure_mtx);    

    eskf::OBSERV_data cur_observ;
        cur_observ.p = Eigen::Vector3d(odom_in->pose.pose.position.x,
                                                                        odom_in->pose.pose.position.y,
                                                                        odom_in->pose.pose.position.z);
        cur_observ.heading.w() = odom_in->pose.pose.orientation.w;
        cur_observ.heading.x() = odom_in->pose.pose.orientation.x;
        cur_observ.heading.y() = odom_in->pose.pose.orientation.y;
        cur_observ.heading.z() = odom_in->pose.pose.orientation.z;
        cur_observ.heading_available = true;
        cur_observ.stamp = odom_in->header.stamp.toSec();

    pose_enu_buff_.push(cur_observ);
}

inline Eigen::Quaterniond quatSlerpInterpolate(const Eigen::Quaterniond & q_front,const Eigen::Quaterniond& q_back,double stamp_frt,double stamp_bk,double stamp_cur){
            double duration = stamp_bk - stamp_frt;
            double offset = stamp_cur - stamp_frt;
            double ratio = offset / duration;
            return q_front.slerp(1-ratio,q_back);
}

/* IMU回调函数 */
void imu_cbk(const sensor_msgs::ImuConstPtr& imu_in)
{
    std::lock_guard<std::mutex> lck(measure_mtx);
    sensor_msgs::Imu imu_cur(*imu_in);
    imuMsg_buff_.push(imu_cur);

    if(eskf_initialized) return;

    if(pose_enu_buff_.size()<5 || imuMsg_buff_.size()<50) return;
        //sychronize imu and observation data
        while(!pose_enu_buff_.empty()){
                if(imuMsg_buff_.front().header.stamp.toSec() > pose_enu_buff_.front().stamp){
                    pose_enu_buff_.pop();
                    continue;
                }else{
                    break;
                }
        }
        if(pose_enu_buff_.empty()) return;

        while(!imuMsg_buff_.empty()){
            if(imuMsg_buff_.front().header.stamp.toSec() <= pose_enu_buff_.front().stamp){
                imu_last = imuMsg_buff_.front();
                imuMsg_buff_.pop();
            }else{
                break;
            }
        }
        if(imuMsg_buff_.empty()) return;

        //State Initialization
        Eigen::Quaterniond q_front(imu_last.orientation.w,
                                                                imu_last.orientation.x,
                                                                imu_last.orientation.y,
                                                                imu_last.orientation.z);
        Eigen::Quaterniond q_back(imuMsg_buff_.front().orientation.w,
                                                                imuMsg_buff_.front().orientation.x,
                                                                imuMsg_buff_.front().orientation.y,
                                                                imuMsg_buff_.front().orientation.z);
        Eigen::Quaterniond q_init = quatSlerpInterpolate(q_front,q_back,
                                                            imu_last.header.stamp.toSec(),imuMsg_buff_.front().header.stamp.toSec(),pose_enu_buff_.front().stamp);
                                                q_init.normalize();
        Eigen::Vector3d p_init(pose_enu_buff_.front().p);

        eskf::State init_state;
            init_state.q = q_init;
            init_state.p = p_init;
            init_state.v = Eigen::Vector3d::Zero();//suppose starting from static state;
            init_state.ba = Eigen::Vector3d::Zero();
            init_state.bg = Eigen::Vector3d::Zero();
            init_state.g = Eigen::Vector3d(0,0,-9.81);
            init_state.stamp = pose_enu_buff_.front().stamp;

        state_estimator->specify_init_state(init_state);
        eskf_initialized = true;
        ROS_INFO("\033[1;36mESKF Initialized: [%0.2f, %0.2f, %0.2f] with current heading %0.1f degree w.r.t local ENU defined by FCU.\033[0m",
                                p_init(0),p_init(1),p_init(2),acosf(q_init.w())*180/M_PI_2);//suppose rotation vector Z is parallel to gravity direction up to sky
}

bool sync_measurement(eskf::Measure &measure_group_){
            std::lock_guard<std::mutex> lck(measure_mtx);
            if(imuMsg_buff_.empty()){
                return false;
            }//如果连imu消息都没有，就不能评估当前状态，直接返回

            double cur_timestamp = ros::Time::now().toSec();

            /* std::vector<double> stamp_queue;
            stamp_queue.reserve(imuMsg_buff_.size());
            std::vector<Eigen::Quaternionf> quat_queue;
            quat_queue.reserve(imuMsg_buff_.size()); */

            while(!imuMsg_buff_.empty() && imuMsg_buff_.front().header.stamp.toSec()<cur_timestamp){
                eskf::IMU_data imu_;
                imu_.angular_velo = Eigen::Vector3d(imuMsg_buff_.front().angular_velocity.x,
                                                                                            imuMsg_buff_.front().angular_velocity.y,
                                                                                            imuMsg_buff_.front().angular_velocity.z);
                imu_.linear_acc = Eigen::Vector3d(imuMsg_buff_.front().linear_acceleration.x,
                                                                                            imuMsg_buff_.front().linear_acceleration.y,
                                                                                            imuMsg_buff_.front().linear_acceleration.z);
                imu_.stamp = imuMsg_buff_.front().header.stamp.toSec();
                measure_group_.imu_buf.push_back(imu_);

                /* stamp_queue.push_back(imu_.stamp);
                quat_queue.push_back(Eigen::Quaternionf(imuMsg_buff_.front().orientation.w,imuMsg_buff_.front().orientation.x,imuMsg_buff_.front().orientation.y,imuMsg_buff_.front().orientation.z)); */
                
                imuMsg_buff_.pop();
            }

            if(pose_enu_buff_.empty()){
                return true;
            }//直接返回，只做预测

            //pose_enu_buff_非空，则先判断最后一帧imu是否能覆盖
            double cur_pose_stamp = pose_enu_buff_.front().stamp;
            double imu_end_stamp = measure_group_.imu_buf.back().stamp;
            if(cur_pose_stamp>measure_group_.imu_buf.back().stamp){
                // cout<<"probably miss some imu messages!"<<endl;
                return true;//不能覆盖就暂时先不考虑pose correction
            }

            while(!pose_enu_buff_.empty()){
                if(pose_enu_buff_.front().stamp>cur_timestamp) break;
                if(pose_enu_buff_.front().stamp>imu_end_stamp) break;

                measure_group_.observ_data.push_back(pose_enu_buff_.front());
                pose_enu_buff_.pop();
            }
            //这里主要是找到与GPS时间上最近的一帧imu的姿态，没有球面插值主要是为了简化运算
            /* for(size_t i=0; i<measure_group_.observ_data.size();i++){
                size_t j=0;
                for(auto &iter : stamp_queue){
                    if(iter > measure_group_.observ_data[i].stamp) {
                        break;
                    }
                    j++;
                }
                if(j<quat_queue.size()-1){           
                    measure_group_.observ_data[i].heading = quat_queue[j];
                    measure_group_.observ_data[i].heading_available =true;
                }
            } */
            return true;
        }

void state_to_odom_msg(eskf::State _state){
            nav_msgs::Odometry odom_msg_;
            odom_msg_.header.frame_id = "odom";
            odom_msg_.child_frame_id = "base_link";
            odom_msg_.header.stamp = ros::Time(_state.stamp);

            odom_msg_.pose.pose.position.x = _state.p[0];
            odom_msg_.pose.pose.position.y = _state.p[1];
            odom_msg_.pose.pose.position.z = _state.p[2];

            odom_msg_.pose.pose.orientation.w = _state.q.w();
            odom_msg_.pose.pose.orientation.x = _state.q.x();
            odom_msg_.pose.pose.orientation.y = _state.q.y();
            odom_msg_.pose.pose.orientation.z = _state.q.z();
            
            Eigen::Vector3d v_body = _state.q.inverse() * _state.v;
            odom_msg_.twist.twist.linear.x = v_body[0];
            odom_msg_.twist.twist.linear.y = v_body[1];
            odom_msg_.twist.twist.linear.z = v_body[2];
            
            Eigen::Vector3d r_body = _state.q.inverse() * _state.angular_velo;
            odom_msg_.twist.twist.angular.x = r_body[0];
            odom_msg_.twist.twist.angular.y = r_body[1];
            odom_msg_.twist.twist.angular.z = r_body[2];
            filtered_odom_pub_.publish(odom_msg_);
}

    /* 接收来自FCU的EKF原点 */
void setLocalENUOriginLLA(const sensor_msgs::NavSatFixConstPtr& msg){
        if(fcu_ekf_origin_offset_received) return;

        // geo_converter.Reset(msg->latitude,msg->longitude,msg->altitude);
        geo_converter.Reset(31.02868866,121.43802274,0);
        fcu_ekf_origin_offset_received = true;

        gps_trajectory.header.frame_id = "odom";
        gps_trajectory.header.stamp = msg->header.stamp;
        gps_trajectory.poses.reserve(10000);

        ROS_INFO("\033[1;36mFCU EKF Origin Set:[%0.3f %0.3f %0.3f].\033[0m",msg->latitude,msg->longitude,msg->altitude);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "usv_eskf_estimator");
    ros::NodeHandle nh;
    ROS_INFO("\033[1;32m----> State Estimator Started.\033[0m");

    fcu_ekf_orig_sub_ = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/ekf_origin",10,&setLocalENUOriginLLA);
    imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/imu/data",2000,&imu_cbk,ros::TransportHints().tcpNoDelay());
    int pose_source_;
    nh.param<int>("eskf/Pose_Source",pose_source_,0);//0 for gps, 1 for slam,etc.
    switch (pose_source_){
        case 0:
        {
            gps_sub_ = nh.subscribe<sensor_msgs::NavSatFix>("/gps",100,&gps_cbk,ros::TransportHints().tcpNoDelay());
            break;
        }

        case 1:
        {
            odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom",100,&odom_cbk,ros::TransportHints().tcpNoDelay());
            break;
        }

        default:
        {
            ROS_WARN("Error Pose Source!");    
            break;
        }
    }

    filtered_odom_pub_ = nh.advertise<nav_msgs::Odometry>("eskf/odometry/filtered",1000);
    gps_path_pub_ = nh.advertise<nav_msgs::Path>("eskf/trajectory/gps",100);

    state_estimator.reset(new eskf::eskf(nh)); 
    
    ros::Rate rate(5000);
    while(ros::ok()){
        ros::spinOnce();
        if(eskf_initialized) {
            if(sync_measurement(cur_measure)){
                state_estimator ->inputMeasure(cur_measure);

                cur_state = state_estimator->get_state(); // 位置姿态、包括角度和角速度

                state_to_odom_msg(cur_state);
            }//sync_measurement
        }//eskf_initialized
        rate.sleep();
    }

    return 0;
}