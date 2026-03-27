#include <eskf_estimator/eskf_estimator.h>

namespace eskf{
eskf::eskf(ros::NodeHandle &nh){
    // dataset = _node["dataset"].as<std::string>();
    node_ = nh;
    std::vector<double> state_std_vec;
    node_.param<vector<double>>("eskf/init_state_std", state_std_vec, vector<double>());
    state_std.setZero();
    state_std.diagonal() = Eigen::Map<Eigen::Matrix<double,18,1>>(state_std_vec.data());
    node_.param<double>("eskf/imuAccNoise",noise_a,0.0);
    node_.param<double>("eskf/imuAccBiasN",noise_ba,0.0);
    node_.param<double>("eskf/imuGyrNoise",noise_g,0.0);
    node_.param<double>("eskf/imuGyrBiasN",noise_bg,0.0);

    node_.param<double>("eskf/xCovariance",v_x,1.0);
    node_.param<double>("eskf/yCovariance",v_y,1.0);
    node_.param<double>("eskf/zCovariance",v_z,1.0);
    node_.param<double>("eskf/magCovariance",v_r,0.01);
}

void eskf::specify_init_state(const State& _init_state){
    state = _init_state;
}

/*
此函数相当于eskf的回调函数
*/
void eskf::inputMeasure(Measure &_measure){
    while(!_measure.observ_data.empty()){
        if(_measure.observ_data.front().stamp - state.stamp < -0.1){
            ROS_WARN("Coming observation data stamp earlier than current state.Time Seq Error.");
            // _measure.observ_data.pop_front();
            // continue;
        } 
        prediction(_measure.imu_buf,_measure.observ_data.front().stamp);

        if(_measure.observ_data.front().heading_available){
            correction(_measure.observ_data.front().p,_measure.observ_data.front().heading);
        }else{
            correction(_measure.observ_data.front().p);
        }

        _measure.observ_data.pop_front();
    }

    //对可能剩余的imu帧继续进行状态预测
    if(!_measure.imu_buf.empty()){
        prediction(_measure.imu_buf,ros::Time::now().toSec());
        // printf("Prediction Finished\n");
    }
}
 
void eskf::correction(Eigen::Vector3d _observation_cartesian,Eigen::Quaterniond _observation_quat){
    ///FINISHED: 添加关于imu姿态的校正
    //观测矩阵的维度为 18*6
    //calculate delta_q, as quaternion is over-parametrized, only q_x,q_y,q_z are considered here to obtain jaccobian of H w.r.t. delta_q
        Eigen::Quaterniond q_nominal = state.q;
        Eigen::Quaterniond delta_q = q_nominal.conjugate()*_observation_quat;
        Eigen::Vector3d delta_rot(delta_q.x(),delta_q.y(),delta_q.z());//first order approximation of quaternion
        delta_rot *= 2;
    
        Eigen::Matrix<double,6,18> H_matrix_ = Eigen::Matrix<double,6,18>::Zero();
        H_matrix_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
        //这里参考了《Quaternion kinematics for the error-state KF》P39的公式
        H_matrix_.block<3,3>(3,6) = 0.5*(BuildSkewMatrix(Eigen::Vector3d(q_nominal.x(),q_nominal.y(),q_nominal.z())) + q_nominal.w()*Eigen::Matrix3d::Identity());
        // cout<<H_matrix_<<endl;
        Eigen::Matrix<double,6,6> V_matrix_;
            V_matrix_ << v_x,0,0,0,0,0,
                                        0,v_y,0,0,0,0,
                                        0,0,v_z,0,0,0,
                                        0,0,0,v_r,0,0,
                                        0,0,0,0,v_r,0,
                                        0,0,0,0,0,v_r;
        // cout<<V_matrix_<<endl;
        Eigen::Matrix<double,18,6> kalman_gain_ = state_std * H_matrix_.transpose() * ((H_matrix_ * state_std * (H_matrix_.transpose()) + V_matrix_).inverse());
        // cout<<kalman_gain_<<endl;
        Eigen::Matrix<double,6,1> delta_y;
        delta_y.block<3,1>(0,0) =  _observation_cartesian - state.p;
        delta_y.block<3,1>(3,0) = delta_rot;
        // cout<<delta_y<<endl;
        Eigen::Matrix<double,18,1> delta_x_ = kalman_gain_ * delta_y;
        // cout<<delta_x_<<endl;
        state_std = (Eigen::Matrix<double,18,18>::Identity() - kalman_gain_ * H_matrix_) * state_std;

    //进行状态的更新 将误差状态叠加到nominal state上
    Eigen::Vector3d error_state_rot = delta_x_.block<3,1>(6,0);
    double error_state_rot_norm = error_state_rot.norm();
    if(error_state_rot_norm > 0.0000001){
        error_state_rot /= error_state_rot_norm;
        error_state_rot *= std::sin(error_state_rot_norm/2);
        Eigen::Quaterniond error_state_q_(std::cos(error_state_rot_norm/2),error_state_rot[0],error_state_rot[1],error_state_rot[2]);
        state.q = state.q * error_state_q_;
    }

    state.q.normalize();
    state.p += delta_x_.block<3,1>(0,0);
    state.v += delta_x_.block<3,1>(3,0);
    state.bg += delta_x_.block<3,1>(9,0);
    state.ba += delta_x_.block<3,1>(12,0);
    state.g +=  delta_x_.block<3,1>(15,0);

    //ESKF reset
    Eigen::Matrix3d G_matrix_;
    G_matrix_ = Eigen::Matrix3d::Identity()-BuildSkewMatrix(error_state_rot*0.5);
    Eigen::Matrix3d temp_matrix_;
    temp_matrix_ = G_matrix_*state_std.block<3,3>(6,6)*G_matrix_.transpose(); 
    state_std.block<3,3>(6,6) = temp_matrix_;
}

void eskf::correction(Eigen::Vector3d _observation_cartesian){
    
    //观测矩阵的维度为 18*3
    Eigen::Matrix<double,3,18> H_matrix_ = Eigen::Matrix<double,3,18>::Zero();
    H_matrix_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d V_matrix_;
    V_matrix_ << v_x,0,0,
                 0,v_y,0,
                 0 ,0,v_z;
    Eigen::Matrix<double,18,3> kalman_gain_ = state_std * H_matrix_.transpose() * ((H_matrix_ * state_std * (H_matrix_.transpose()) + V_matrix_).inverse());
    Eigen::Matrix<double,18,1> delta_x_ = kalman_gain_ * (_observation_cartesian - state.p);
    state_std = (Eigen::Matrix<double,18,18>::Identity() - kalman_gain_ * H_matrix_) * state_std;

    //进行状态的更新 将误差状态叠加到nominal state上
    Eigen::Vector3d error_state_rot = delta_x_.block<3,1>(6,0);
    double error_state_rot_norm = error_state_rot.norm();
    if(error_state_rot_norm > 0.0000001){
        error_state_rot /= error_state_rot_norm;
        error_state_rot *= std::sin(error_state_rot_norm/2);
        Eigen::Quaterniond error_state_q_(std::cos(error_state_rot_norm/2),error_state_rot[0],error_state_rot[1],error_state_rot[2]);
        state.q = state.q * error_state_q_;
    }
    state.q.normalize();
    state.p += delta_x_.block<3,1>(0,0);
    state.v += delta_x_.block<3,1>(3,0);
    state.bg += delta_x_.block<3,1>(9,0);
    state.ba += delta_x_.block<3,1>(12,0);
    state.g +=  delta_x_.block<3,1>(15,0);

    //ESKF reset
    // Eigen::Matrix3d G_matrix_;
    // G_matrix_ = Eigen::Matrix3d::Identity()-BuildSkewMatrix(error_state_rot*0.5);
    // Eigen::Matrix3d temp_matrix_;
    // temp_matrix_ = G_matrix_*state_std.block<3,3>(6,6)*G_matrix_.transpose(); 
    // state_std.block<3,3>(6,6) = temp_matrix_;
}

void eskf::prediction(std::deque<IMU_data>& _imu_buf, double stop_before){
    /*
    根据 当前最新的measure 以及 当前的状态  进行IMU惯性结算 获得nominal state 
    */
   while(!_imu_buf.empty() && _imu_buf.front().stamp<stop_before){
        IMU_data imu_data_;
        if(last_imu_ == nullptr){
            imu_data_ = _imu_buf.front();
        }else{
            imu_data_.linear_acc = (last_imu_->linear_acc + _imu_buf.front().linear_acc)/2;
            imu_data_.angular_velo = (last_imu_->angular_velo + _imu_buf.front().angular_velo)/2;
            imu_data_.stamp = min(_imu_buf.front().stamp,stop_before);
        }
        forward_propagation(imu_data_);

        last_imu_.reset(new IMU_data(_imu_buf.front()));
        _imu_buf.pop_front();
   }
}

void eskf::forward_propagation( IMU_data _imu_data){
    /*
    将当前状态 state 上根据_imu_data进行惯性解算 
    其中需要进行状态递推的时间gap = _imu_data.stamp - state.stamp;
    */
    _imu_data.angular_velo -= state.bg;
    _imu_data.linear_acc -= state.ba;
    double time_gap = _imu_data.stamp - state.stamp;
    // printf("time gap %0.6f\n",time_gap);

    state.p += state.v * time_gap;
    state.v += (state.q * _imu_data.linear_acc + state.g) * time_gap;
    state.q = state.q * Exp(_imu_data.angular_velo,time_gap);
    state.q.normalize();
    state.angular_velo = state.q *_imu_data.angular_velo;
    state.stamp = _imu_data.stamp;

    // 在此处完成状态方差的递推
    Eigen::Matrix<double,18,18> fx_ = make_Fx_matrix_from_imu_and_delta_t(_imu_data,time_gap);
    Eigen::Matrix<double,18,18> Q_ = make_Q_matrix_from_imu_and_delta_t(_imu_data,time_gap);
    state_std = fx_ * state_std * fx_.transpose() + Q_;
    // printf("X: %0.6f Y: %0.6f Z: %0.6f Qw: %0.6f Qx: %0.6f Qy: %0.6f Qz: %0.6f\n",state.p.x(),state.p.y(),state.p.z(),state.q.w(),state.q.x(),state.q.y(),state.q.z());
}

Eigen::Matrix<double,18,18> eskf::make_Q_matrix_from_imu_and_delta_t(IMU_data _imu_data,double _delta_t){

    //q以及g的噪声为0 ， 只需要对其余的项进行设置   
    Eigen::Matrix<double,18,18> Q_matrix_ = Eigen::Matrix<double,18,18>::Zero();
    
    Q_matrix_.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * (_delta_t * noise_a) * (_delta_t * noise_a);
    Q_matrix_.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * (_delta_t * noise_g) * (_delta_t * noise_g);
    Q_matrix_.block<3,3>(9,9) = Eigen::Matrix3d::Identity() * (_delta_t * noise_bg * noise_bg);
    Q_matrix_.block<3,3>(12,12) = Eigen::Matrix3d::Identity() * (_delta_t * noise_ba * noise_ba);

    return Q_matrix_;

}

//Fx的构造方式参考高博的ESKF知乎帖子
//! 高博的Fx矩阵有点问题 bg  ba 的项搞混了 ,状态量和上文的公式不太一样，帖子中的F矩阵对应的状态量顺序为 p v q ba bg g
Eigen::Matrix<double,18,18> eskf::make_Fx_matrix_from_imu_and_delta_t(IMU_data _imu_data,double _delta_t){
    Eigen::Matrix<double,18,18> Fx_ = Eigen::Matrix<double,18,18>::Identity();
    Fx_.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * _delta_t;
    Fx_.block<3,3>(3,6) = -state.q.toRotationMatrix() * BuildSkewMatrix(_imu_data.linear_acc) * _delta_t;
    Fx_.block<3,3>(3,12) = -state.q.toRotationMatrix() * _delta_t;
    Fx_.block<3,3>(3,15) = state.q.toRotationMatrix() * _delta_t;
    Fx_.block<3,3>(6,6) = Exp(-_imu_data.angular_velo,_delta_t);
    Fx_.block<3,3>(6,9) = -Eigen::Matrix3d::Identity() * _delta_t;
    return Fx_;
}

Eigen::Matrix3d eskf::BuildSkewMatrix(const Eigen::Vector3d& vec){
    Eigen::Matrix3d matrix;
    matrix << 0.0,     -vec[2],   vec[1],
              vec[2],    0.0,     -vec[0],
              -vec[1],   vec[0],    0.0;

    return matrix;
}

//I copied it from FAST-LIO lol. Thanks to HKU.
Eigen::Matrix3d eskf::Exp(const Eigen::Vector3d &ang_vel, const double &dt){
    double ang_vel_norm = ang_vel.norm();
    Eigen::Matrix3d Eye3 = Eigen::Matrix3d::Identity();
    if (ang_vel_norm > 0.0000001)
    {
        Eigen::Vector3d r_axis = ang_vel / ang_vel_norm; 
        Eigen::Matrix3d K = BuildSkewMatrix(r_axis); 
        double r_ang = ang_vel_norm * dt; 
        /// Roderigous Tranformation
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

}