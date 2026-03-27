#include <dynamic_occupancy_grid.h>


#define logit(x) (log((x) / (1 - (x))))
#define prob(x) (1-1/(1+exp(x)))

#define deg2Rad M_PI/180.0

using namespace std;

// #define DEBUG
//2D mapping data
DynamicOccupancyGrid::DynamicOccupancyGrid(ros::NodeHandle &nh){
            node_ = nh;
            node_.param<double>("DynamicOccupancyGrid/mapLength",map_length,50.0);
            node_.param<double>("DynamicOccupancyGrid/mapResolution",map_resolution_,0.2);
            node_.param<double>("DynamicOccupancyGrid/maxRange",max_range,100.0);
            node_.param<int>("DynamicOccupancyGrid/horizontalScanNum",horizontal_scan_num,360);
            node_.param<double>("DynamicOccupancyGrid/sensorHeightLimitUpper",sensor_height_limit_upper,0.5);
            node_.param<double>("DynamicOccupancyGrid/sensorHeightLimitDown",sensor_height_limit_down,-0.8);
            node_.param<double>("DynamicOccupancyGrid/sensorPitch",sensor_pitch,15.0);
            node_.param<bool>("DynamicOccupancyGrid/ifCloudNeedTransform",if_cloud_need_transform,true);
            /* == initialize map parameters == */
            map_resolution_inv_ = 1 / map_resolution_;
            map_length_grid = ceil(map_length*map_resolution_inv_);

            ros_map.info.width = map_length_grid;
            ros_map.info.height = map_length_grid;
            ros_map.info.resolution = map_resolution_;
            ros_map.info.origin.position.x = INFINITY;
            ros_map.info.origin.position.y = INFINITY;
            ros_map.info.origin.position.z = 0.0;

            occupancy_grid_ = vector<double>(map_length_grid*map_length_grid, clamp_min_log_ - unknown_flag_);
            count_hit_ = vector<short>(map_length_grid*map_length_grid,0);
            count_hit_and_miss_ = vector<short>(map_length_grid*map_length_grid,0);

            //PHD filter update
            params_.size = (float)map_length;
            params_.resolution = map_resolution_;
            node_.param<int>("DynamicOccupancyGrid/PHDFilter/particles/particle_count", params_.particle_count, 300000);
            node_.param<int>("DynamicOccupancyGrid/PHDFilter/particles/new_born_particle_count", params_.new_born_particle_count, 30000);
            node_.param<float>("DynamicOccupancyGrid/PHDFilter/particles/persistence_probability", params_.persistence_prob, 0.99f);
            node_.param("DynamicOccupancyGrid/PHDFilter/particles/process_noise_position", params_.stddev_process_noise_position, 0.1f);
            node_.param("DynamicOccupancyGrid/PHDFilter/particles/process_noise_velocity", params_.stddev_process_noise_velocity, 1.0f);
            node_.param("DynamicOccupancyGrid/PHDFilter/particles/birth_probability", params_.birth_prob, 0.025f);
            node_.param("DynamicOccupancyGrid/PHDFilter/particles/stddev_velocity", params_.stddev_velocity, 15.0f);
            node_.param("DynamicOccupancyGrid/PHDFilter/particles/init_max_velocity", params_.init_max_velocity, 15.0f);
            node_.param("DynamicOccupancyGrid/PHDFilter/particles/freespace_discount", params_.freespace_discount, 0.01f);

            laser_params_.fov = 360.0f;
            laser_params_.max_range = max_range;
            node_.param("DynamicOccupancyGrid/PHDFilter/laser/stddev_range", laser_params_.stddev_range, 0.5f);
            laser_params_.resolution = map_resolution_;
            angle_increment = laser_params_.fov / horizontal_scan_num * deg2Rad;
            angle_max = laser_params_.fov / 2 * deg2Rad;
            angle_min = -angle_max;

            node_.param<float>("DynamicOccupancyGrid/PHDFilter/predict/minVelocityThreshold",minVelocityThres,1.0f);
            node_.param<float>("DynamicOccupancyGrid/PHDFilter/predict/timeSpan",dt_predict,2.0f);
            collision_point_sub_ = node_.subscribe<geometry_msgs::Point>("/collision_point",10,&DynamicOccupancyGrid::collisionPointCallback,this);
            if_predict_need = false;

            //initialize PHDFilter things
            grid_generator_.reset(new LaserMeasurementGrid(laser_params_,params_.size,params_.resolution));
            grid_map_.reset(new dogm::DOGM(params_));

            //rayCasting and bayesian update
            node_.param("DynamicOccupancyGrid/raycast/p_hit", p_hit_, 0.70);
            node_.param("DynamicOccupancyGrid/raycast/p_miss", p_miss_, 0.35);
            node_.param("DynamicOccupancyGrid/raycast/p_min", p_min_, 0.12);
            node_.param("DynamicOccupancyGrid/raycast/p_max", p_max_, 0.97);
            node_.param("DynamicOccupancyGrid/raycast/p_occ", p_occ_, 0.80);

            prob_hit_log_ = logit(p_hit_);
            prob_miss_log_ = logit(p_miss_);
            clamp_min_log_ = logit(p_min_);
            clamp_max_log_ = logit(p_max_);
            min_occupancy_log_ = logit(p_occ_);
            unknown_flag_ = 0.01;
            max_ray_length_ = max_range;

            /* == initialize ros subscriber & publisher == */
            cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_,"/cloud",10,ros::TransportHints().tcpNoDelay()));
            odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_,"/odometry",100,ros::TransportHints().tcpNoDelay()));
            sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(
            SyncPolicyCloudOdom(100), *cloud_sub_, *odom_sub_));
            sync_cloud_odom_->registerCallback(boost::bind(&DynamicOccupancyGrid::cloudOdomCallback, this, _1, _2));

            /* = visualization =  */
            inflated_cloud_to_pub.reset(new pcl::PointCloud<pcl::PointXYZ>());
            downSampleCloud.setLeafSize(map_resolution_,map_resolution_,map_resolution_);            
            nav_map_pub_ = node_.advertise<nav_msgs::OccupancyGrid>("/dynamic_occupancy_grid/occupancy_grid_map", 10);

        }

    /* == Main Loop here == */

void DynamicOccupancyGrid::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& msg_cloud,const nav_msgs::OdometryConstPtr& msg_odom){

        auto time1 = std::chrono::high_resolution_clock::now();

        egoMotionCompensation(msg_odom->pose.pose.position);

        /* = preprocess pointcloud = */
        static Eigen::Quaterniond q_pitch(cos(sensor_pitch*deg2Rad/2),0.0,sin(sensor_pitch*deg2Rad/2),0.0);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_global(new pcl::PointCloud<pcl::PointXYZI>);
        if(if_cloud_need_transform){
            Eigen::Quaterniond robot_q(msg_odom->pose.pose.orientation.w,
                                                                            msg_odom->pose.pose.orientation.x,
                                                                            msg_odom->pose.pose.orientation.y,
                                                                            msg_odom->pose.pose.orientation.z);
            Eigen::Matrix4d trans_enu_local;
            trans_enu_local.block<3,3>(0,0) = robot_q.toRotationMatrix()*q_pitch.toRotationMatrix();
            trans_enu_local.block<3,1>(0,3) = Eigen::Vector3d(msg_odom->pose.pose.position.x,msg_odom->pose.pose.position.y,0.0);
            pcl::PointCloud<pcl::PointXYZI> cloud_to_update_;
            cloud_header = msg_cloud->header;
            pcl::fromROSMsg(*msg_cloud,cloud_to_update_);
            pcl::transformPointCloud(cloud_to_update_,*cloud_global,trans_enu_local,true);
        }else{
            cloud_header = msg_cloud->header;
            pcl::fromROSMsg(*msg_cloud,*cloud_global);
        }
        
        if(cloud_global->size() <5) return;

        if(dt<0){
            dt = 0.1;
        }else{
            dt = msg_cloud->header.stamp.toSec() - cloud_header.stamp.toSec();
        }
        dt = 0.1;
        cloud_header = msg_cloud->header;

        project2ScanMeasurements(cloud_global);

        /* = map update = */
        #ifdef RAYCAST
        raycastUpdate();        
        #else
        dogmUpdate();

        #endif

#ifdef DEBUG
        auto time_end = std::chrono::high_resolution_clock::now();
        auto duration_occ = std::chrono::duration_cast<std::chrono::milliseconds>(time_end-time1);
        cout<<"Occupancy map update  cost: "<<duration_occ.count()<<" ms"<<endl;
#endif
        ros_map.header = msg_odom->header;
        ros_map.info.map_load_time = ros_map.header.stamp;

        nav_map_pub_.publish(ros_map);

    }

void DynamicOccupancyGrid::collisionPointCallback(const geometry_msgs::PointConstPtr& msg){
    collision_point = *msg;
    if_predict_need = true;
}

void DynamicOccupancyGrid::egoMotionCompensation(const geometry_msgs::Point & position){
    double origin_x_new = position.x - map_length/2;
    double origin_y_new = position.y - map_length/2;
    if(ros_map.info.origin.position.x == INFINITY){
        ros_map.info.origin.position.x = origin_x_new;
        ros_map.info.origin.position.y = origin_y_new;
        return;
    }
    #ifdef RAYCAST
    int x_move = static_cast<int>((ros_map.info.origin.position.x - origin_x_new)*map_resolution_inv_);
    int y_move = static_cast<int>((ros_map.info.origin.position.y - origin_y_new)*map_resolution_inv_);
    if(pow(x_move,2)+pow(y_move,2)<1)
    {
        return;
    }

    std::vector<short> count_hit_temp = vector<short>(map_length_grid*map_length_grid,0);
    std::vector<short> count_hit_and_miss_temp = vector<short>(map_length_grid*map_length_grid,0);
    std::vector<double>occupancy_grid_temp = vector<double>(map_length_grid*map_length_grid, clamp_min_log_ - unknown_flag_);

    for(int ix = 0; ix < ros_map.info.width;ix++){
        for(int iy = 0; iy < ros_map.info.height;iy++){
            int idx = ix+iy*ros_map.info.width;
            int new_y = iy - y_move;
            int new_x = ix - x_move;
            int new_idx = new_x +new_y*ros_map.info.width;

            if (new_x >= 0 && new_x < ros_map.info.width && new_y >= 0 && new_y < ros_map.info.height)
            {
                count_hit_temp[idx] = count_hit_[new_idx];
                count_hit_and_miss_temp[idx] = count_hit_and_miss_[new_idx];
                occupancy_grid_temp[idx] = occupancy_grid_[new_idx];
            }
        }
    }
    count_hit_.swap(count_hit_temp);
    count_hit_and_miss_.swap(count_hit_and_miss_temp);
    occupancy_grid_.swap(occupancy_grid_temp);
    #endif

    ros_map.info.origin.position.x = origin_x_new;
    ros_map.info.origin.position.y = origin_y_new;
}

void DynamicOccupancyGrid::project2ScanMeasurements(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudIn){
        measurements.clear();
        measurements.resize(horizontal_scan_num,INFINITY);
        double map_center_x  = ros_map.info.origin.position.x + map_length/2;
        double map_center_y  = ros_map.info.origin.position.y + map_length/2;
        //project to 2d scan sector
        for(size_t i =0; i < cloudIn->points.size(); i++){

            double diff_x = cloudIn->points[i].x - map_center_x;
            double diff_y = cloudIn->points[i].y - map_center_y;

            double range = sqrt(pow(diff_x,2)+pow(diff_y,2));
            double height_diff = cloudIn->points[i].z;

            if(height_diff < sensor_height_limit_upper && height_diff >sensor_height_limit_down )
            {
                
                double hori_angle = atan2(diff_y,diff_x);

                while (hori_angle<-M_PI) {
                    hori_angle+=2*M_PI;
                }
                    while (hori_angle>M_PI){
                    hori_angle-=2*M_PI;
                }

                int index =  static_cast<int>((hori_angle - angle_min) / angle_increment);
                boundAngleIndex(index);

                if(range < measurements[index]){
                        measurements[index] = range;
                }
            }
        }
    }

void DynamicOccupancyGrid::raycastUpdate(){
        
        double map_center_x  = ros_map.info.origin.position.x + map_length/2;
        double map_center_y  = ros_map.info.origin.position.y + map_length/2;

        double length;
        double cur_angle;

        RayCaster raycaster;
        Eigen::Vector3d ray_pt;
        Eigen:: Vector2d pt_w;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for(size_t i = 0; i<measurements.size();i++){
            // if( measurements[i] > max_range)
            //     continue;

            cur_angle = (static_cast<float>(i)+0.5)*angle_increment + angle_min;
            length = measurements[i];

            if(length > max_ray_length_){
                pt_w(0) = max_ray_length_ * cos(cur_angle) + map_center_x;
                pt_w(1) = max_ray_length_ * sin(cur_angle) + map_center_y;
                setCacheOccupancy(pt_w,0);
            }else{
                pt_w(0) = length * cos(cur_angle) + map_center_x;
                pt_w(1) = length * sin(cur_angle) + map_center_y;
                setCacheOccupancy(pt_w,1);
            }
            // pcl::PointXYZ p;
            // p.x = pt_w(0);
            // p.y = pt_w(1);
            // p.z =0.0;
            // cloud->push_back(p);

            Eigen::Vector3d pt_w_raycast;
            pt_w_raycast(0) = floor(pt_w(0)*map_resolution_inv_);
            pt_w_raycast(1) = floor(pt_w(1)*map_resolution_inv_);
            pt_w_raycast(2) = 0;

            Eigen::Vector3d robot_pose_raycast;
            robot_pose_raycast(0) = floor(map_center_x*map_resolution_inv_);
            robot_pose_raycast(1) = floor(map_center_y*map_resolution_inv_);
            robot_pose_raycast(2) = 0.0;

            raycaster.setInput(robot_pose_raycast,pt_w_raycast);

            while(raycaster.step(ray_pt))
            {
                Eigen::Vector2d tmp ;
                tmp(0) = (ray_pt(0)+0.5)*map_resolution_;
                tmp(1) = (ray_pt(1)+0.5)*map_resolution_;
                // length = (tmp - robot_pos_2d).norm();

                // if (length < mp_.min_ray_length_) break;

                setCacheOccupancy(tmp, 0);
            }
        }

        while (!cache_voxel_.empty()) {

            Eigen::Vector2i idx = cache_voxel_.front();
            int idx_ctns = toAddress(idx);
            cache_voxel_.pop();

            double log_odds_update =
                count_hit_[idx_ctns] >= count_hit_and_miss_[idx_ctns] - count_hit_[idx_ctns] ?
                prob_hit_log_ :
                prob_miss_log_;

            count_hit_[idx_ctns] = count_hit_and_miss_[idx_ctns] = 0;

            if (log_odds_update >= 0 && occupancy_grid_[idx_ctns] >= clamp_max_log_) {
                continue;
            } else if (log_odds_update <= 0 && occupancy_grid_[idx_ctns] <= clamp_min_log_) {
                occupancy_grid_[idx_ctns] = clamp_min_log_;
                continue;
            }

            occupancy_grid_[idx_ctns] =
                std::min(std::max(occupancy_grid_[idx_ctns] + log_odds_update, clamp_min_log_),
                        clamp_max_log_);
        }

        ros_map.data.clear();
        ros_map.data.resize(map_length_grid*map_length_grid,-1);
        #pragma omp parallel for
        for(int i = 0; i<occupancy_grid_.size();i++){
            ros_map.data[i] = static_cast<int>(prob(occupancy_grid_[i]) * 100.0f);
        }
    }

void DynamicOccupancyGrid::dogmUpdate(){
    double map_center_x  = ros_map.info.origin.position.x + map_length/2;
    double map_center_y  = ros_map.info.origin.position.y + map_length/2;

    auto meas_grid = grid_generator_->generateGrid(measurements);
    grid_map_->updateGrid(meas_grid,float(map_center_x),float(map_center_y),0.0,dt);
    // const auto cells_with_velocity = computeCellsWithVelocity(*grid_map_,0.7,0.5);
	// computeAndSaveResultImages(*grid_map_, cells_with_velocity, 0);

    auto grid_cell_array = grid_map_->getGridCells();

    ros_map.data.clear();
    ros_map.data.resize(map_length_grid*map_length_grid,-1);
    #pragma omp parallel for
    for (int i = 0; i < grid_cell_array.size(); i++)
    {
        auto cell = grid_cell_array[i];
        float free_mass = cell.free_mass;
        float occ_mass = cell.occ_mass;
        
        float prob = occ_mass + 0.5f * (1.0f - occ_mass - free_mass);

        if (prob == 0.5f)
        {
            ros_map.data[i] = -1;
        }
        else
        {      
            ros_map.data[i] = static_cast<int>(prob * 100.0f);
        } 
    }

    addPredictionLayer(grid_cell_array);
}

void DynamicOccupancyGrid::addPredictionLayer(const std::vector<dogm::GridCell> &grid_cell_array){
    static int pred_count = 0;
    if(!if_predict_need) return;
    if(pred_count > 20){
        pred_count = 0;
        if_predict_need = false;
        return;
    }
    //围绕碰撞点对对占据栅格进行预测
    //1. 确定需要进行预测的地图范围，碰撞点为中心，构建一个10m乘以10m的正方形区域
    int x_lower, x_upper, y_lower, y_upper;
    x_lower = round((collision_point.x - 5.0 - ros_map.info.origin.position.x) * map_resolution_inv_);
    x_upper = round((collision_point.x + 5.0 - ros_map.info.origin.position.x) * map_resolution_inv_);
    y_lower = round((collision_point.y - 5.0 - ros_map.info.origin.position.y) * map_resolution_inv_);
    y_upper = round((collision_point.y + 5.0 - ros_map.info.origin.position.y) * map_resolution_inv_);
    boundIndex(x_lower);
    boundIndex(x_upper);
    boundIndex(y_lower);
    boundIndex(y_upper);

    //2. 获取动态栅格以及预测后的栅格
    std::vector<Eigen::Vector2d> dynamic_grids;
    dynamic_grids.reserve((x_upper - x_lower+1)*(y_upper - y_lower+1));
    std::vector<Eigen::Vector2d> prediction_grids;
    prediction_grids.reserve((x_upper - x_lower+1)*(y_upper - y_lower+1));
    for(int x = x_lower; x<=x_upper;x++){
        for(int y = y_lower; y<=y_upper;y++){
            int id = x+y*map_length_grid;
            auto cell = grid_cell_array[id];

            float free_mass = cell.free_mass;
            float occ_mass = cell.occ_mass;

            float prob = occ_mass + 0.5f * (1.0f - occ_mass - free_mass);
            if(prob < 0.85) continue;

            //计算速度向量的马氏距离
            Eigen::Vector2f vel;
            vel << cell.mean_x_vel, cell.mean_y_vel;

            Eigen::Matrix2f covar;
            covar << cell.var_x_vel, cell.covar_xy_vel, cell.covar_xy_vel,
            cell.var_y_vel;

            auto mdist = vel.transpose() * covar.inverse() * vel; 

            if(mdist < minVelocityThres) continue;

            //定义在局部地图坐标系下
            Eigen::Vector2d dyn_grid;
            dyn_grid[0] = (x+0.5)*map_resolution_;
            dyn_grid[1] = (y+0.5)*map_resolution_;

            Eigen::Vector2d pred_grid;
            pred_grid[0] = dyn_grid[0] + dt_predict*vel.x();
            pred_grid[1] = dyn_grid[1] + dt_predict*vel.y();

            dynamic_grids.push_back(dyn_grid);
            prediction_grids.push_back(pred_grid);
        }
    }

    //3. 基于raycast更新
    RayCaster raycaster;
    Eigen::Vector3d ray_pt;
    Eigen::Vector3d pt_raycast_end;
    Eigen::Vector3d pt_raycast_start;
    for(int k = 0; k < dynamic_grids.size(); k++){
        pt_raycast_end(0) = round(prediction_grids[k].x()*map_resolution_inv_);
        pt_raycast_end(1) = round(prediction_grids[k].y()*map_resolution_inv_);
        pt_raycast_end(2) = 0;

        pt_raycast_start(0) = round(dynamic_grids[k].x()*map_resolution_inv_);
        pt_raycast_start(1) = round(dynamic_grids[k].y()*map_resolution_inv_);
        pt_raycast_start(2) = 0.0;

        raycaster.setInput(pt_raycast_start,pt_raycast_end);

        while(raycaster.step(ray_pt))
        {
            int x = ray_pt(0);
            int y = ray_pt(1);
            boundIndex(x);
            boundIndex(y);
            int id = x + y*map_length_grid;
            ros_map.data[id] = int8_t(90);
        }
    }
    std::cout<<"dynamic grids captured num: "<<dynamic_grids.size()<<std::endl;
    pred_count ++;
}

void DynamicOccupancyGrid::setCacheOccupancy(Eigen::Vector2d pos, bool occ){
        int ix = round((pos(0) - ros_map.info.origin.position.x)*map_resolution_inv_);
        boundIndex(ix);
        int iy = round((pos(1) - ros_map.info.origin.position.y)*map_resolution_inv_);
        boundIndex(iy);
        
        int idx = ix+ iy*map_length_grid;

        count_hit_and_miss_[idx] += 1;

        if(count_hit_and_miss_[idx]==1){
            cache_voxel_.push(Eigen::Vector2i(ix,iy));
        }

        if(occ){
            count_hit_[idx] +=1;
        }
        
        return;
    }

inline void DynamicOccupancyGrid::boundIndex(int& id) {
    int id1;
    id1 = max(min(id, map_length_grid - 1), 0);
    id = id1;

    }//限制索引不能超出地图范围

inline void DynamicOccupancyGrid::boundAngleIndex(int& id){
        int id1;
        id1 = max(min(id, horizontal_scan_num - 1), 0);
        id = id1;
}

inline int DynamicOccupancyGrid::toAddress(Eigen::Vector2i buffer_index){
        return buffer_index.y() * ros_map.info.width + buffer_index.x();
    }

/// @brief 在完成map sliding以及resetbuffer操作后，才能更新局部地图的位置信息，否则可能错误地重置Buffer范围，
