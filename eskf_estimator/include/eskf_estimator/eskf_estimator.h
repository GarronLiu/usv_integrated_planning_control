#ifndef ESKF_ESITIMATOR_H
#define ESKF_ESITIMATOR_H
/*  The following code is adapted from [leo6862(github user name)] available at [https://github.com/leo6862/ins_eskf_kitti/tree/master].
# Special thanks to [leo6862] for his valuable open-source contribution.*/

#include <Eigen/Dense>
#include <ros/ros.h>
#include  <iostream>
#include <mutex>
#include <iomanip>
#include <stdlib.h>
#include <deque>
using namespace std;

namespace eskf{
    struct IMU_data{
            double stamp;
            Eigen::Vector3d linear_acc;
            Eigen::Vector3d angular_velo;
        };

    struct OBSERV_data{
            double stamp;
            Eigen::Vector3d p;
            Eigen::Quaterniond heading;
            bool heading_available = false;
        };

    struct State{
            Eigen::Quaterniond q=Eigen::Quaterniond(-10.0,-10.0,-10.0,-10.0);
            Eigen::Vector3d v;
            Eigen::Vector3d p;
            Eigen::Vector3d bg;
            Eigen::Vector3d ba;
            Eigen::Vector3d g;
            Eigen::Vector3d angular_velo;
            double stamp;
        };

    struct Measure{
            std::deque<IMU_data> imu_buf;
            std::deque<OBSERV_data> observ_data;
        };

    class eskf{
        public:
            eskf(ros::NodeHandle &nh);
            void specify_init_state(const State& _init_state);
            inline State get_state() {return state;}
            inline double get_state_stamp() {return state.stamp;}
            Eigen::Matrix3d BuildSkewMatrix(const Eigen::Vector3d& vec);
            void inputMeasure( Measure & _measure);
            
        private:
            void prediction(std::deque<IMU_data>& _imu_buf, double stop_before); //imu 惯性解算获得nomial state.
            void forward_propagation( IMU_data _imu_data);
            void correction(Eigen::Vector3d _observation_cartesian,Eigen::Quaterniond _observation_quat);
            void correction(Eigen::Vector3d _observation_cartesian);
            Eigen::Matrix<double,18,18>  make_Fx_matrix_from_imu_and_delta_t(IMU_data _imu_data,double _delta_t);
            Eigen::Matrix<double,18,18> make_Q_matrix_from_imu_and_delta_t(IMU_data _imu_data,double _delta_t);

            Eigen::Matrix3d Exp(const Eigen::Vector3d &ang_vel, const double &dt);
            
        private:
            ros::NodeHandle node_;
            // double initialization_stamp;
            State state;
            Measure current_measure;
            unique_ptr<IMU_data> last_imu_;

            Eigen::Matrix<double,18,18> state_std; //状态协方差矩阵（预测与更新过程）

            double noise_a = -1; //这个-1是一个标志位  判断是否已经通过YAML::Node给Q中的元素赋值
            double noise_g;
            double noise_ba;
            double noise_bg;

            //variables for V .Observation noise 
            double v_x;
            double v_y;
            double v_z;
            double v_r;

        }; 
}


#endif