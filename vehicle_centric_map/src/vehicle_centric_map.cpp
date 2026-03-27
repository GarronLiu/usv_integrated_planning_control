#include "vehicle_centric_map/vehicle_centric_map.h"

using namespace std;
// 2D mapping data
VehicleCentriMap::VehicleCentriMap(ros::NodeHandle& nh)
{
  node_ = nh;
  node_.param<double>("vehicleCentricMap/obstaclesInflation", obstacles_inflation, 0.4);
  node_.param<int>("vehicleCentricMap/p_occ", p_occ, 85);
  /* == initialize ros subscriber & publisher == */
  nav_map_sub_ = node_.subscribe<nav_msgs::OccupancyGrid>("/dynamic_occupancy_grid/occupancy_grid_map", 10,
                                                          &VehicleCentriMap::occupancyCallback, this,
                                                          ros::TransportHints().tcpNoDelay());
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/vehicle_centric_map/occupancy_inflate", 10);
  esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/vehicle_centric_map/esdf", 10);

  map_init_ = false;
}

void VehicleCentriMap::occupancyCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  auto map = *msg;
  if (!map_init_)
  {
    init(map.info.width, map.info.height, map.info.resolution);
  }
  map_origin_x = map.info.origin.position.x;
  map_origin_y = map.info.origin.position.y;

  clearAndInflateLocalMap(map);

  updateESDF();

  map_init_ = true;

  publishESDF();
  publishMap();
}

void VehicleCentriMap::init(const int map_width, const int map_height, const double resolution)
{
  map_grid_num = map_width * map_height;
  map_length = map_width * resolution;
  map_length_grid = map_width;
  map_resolution_ = resolution;
  map_resolution_inv_ = 1 / map_resolution_;

  std::cout << map_length_grid << " " << map_grid_num << std::endl;
}

void VehicleCentriMap::clearAndInflateLocalMap(const nav_msgs::OccupancyGrid& map)
{
  // clear outdated map
  occupancy_grid_inflate_.clear();
  occupancy_grid_inflate_.resize(map_grid_num, 0);

  // inflate obstacles
  int inflation_step = ceil(obstacles_inflation * map_resolution_inv_);
  vector<int> inf_pts_buffer_id(pow(2 * inflation_step + 1, 2));
  Eigen::Vector2i inf_pt_buffer_id;

  for (int ix = 0; ix < map_length_grid; ix++)
  {
    for (int iy = 0; iy < map_length_grid; iy++)
    {
      int idx = ix + iy * map_length_grid;
      if (map.data[idx] <= 85)
        continue;

      inflatePoint(Eigen::Vector2i(ix, iy), inflation_step, inf_pts_buffer_id);
      for (size_t k = 0; k < inf_pts_buffer_id.size(); k++)
      {
        occupancy_grid_inflate_[inf_pts_buffer_id[k]] = 1;
      }
    }
  }
}

inline void VehicleCentriMap::inflatePoint(const Eigen::Vector2i& pt, int step, vector<int>& pts)
{
  int num = 0;
  for (int ix = pt(0) - step; ix <= pt(0) + step; ix++)
  {
    for (int iy = pt(1) - step; iy <= pt(1) + step; iy++)
    {
      if (ix >= 0 && ix < map_length_grid && iy >= 0 && iy < map_length_grid)
      {
        auto buffer_index_ = ix + iy * map_length_grid;
        pts[num] = buffer_index_;
        num++;
      }
    }
  }
}

void VehicleCentriMap::updateESDF()
{
  occupancy_grid_neg.clear();
  distance_buffer_.clear();
  distance_buffer_neg_.clear();
  distance_buffer_all_.clear();
  tmp_buffer1_.clear();

  occupancy_grid_neg.resize(map_grid_num, 0);
  distance_buffer_.resize(map_grid_num, 10000.0);
  distance_buffer_neg_.resize(map_grid_num, 10000.0);
  distance_buffer_all_.resize(map_grid_num, 10000);
  tmp_buffer1_.resize(map_grid_num, 0);

  /* ========== compute positive DT ========== */
  for (int local_index_x = 0; local_index_x < map_length_grid; local_index_x++)
  {
    fillESDF(
        [&](int local_index_y) {
          auto idx = local_index_x + local_index_y * map_length_grid;
          return occupancy_grid_inflate_[idx] == 1 ? 0 : std::numeric_limits<double>::max();
        },
        [&](int local_index_y, double val) {
          auto idx = local_index_x + local_index_y * map_length_grid;
          tmp_buffer1_[idx] = val;
        },
        0, map_length_grid - 1, 1);
  }
  for (int local_index_y = 0; local_index_y < map_length_grid; local_index_y++)
  {
    fillESDF(
        [&](int local_index_x) {
          auto idx = local_index_x + local_index_y * map_length_grid;
          return tmp_buffer1_[idx];
        },
        [&](int local_index_x, double val) {
          auto idx = local_index_x + local_index_y * map_length_grid;
          distance_buffer_[idx] = map_resolution_ * std::sqrt(val);
          //  min(mp_.resolution_ * std::sqrt(val),
          //      md_.distance_buffer_[toAddress(x, y, z)]);
        },
        0, map_length_grid - 1, 0);
  }
  /* ========== compute negative distance ========== */
  for (int local_index_x = 0; local_index_x < map_length_grid; local_index_x++)
  {
    for (int local_index_y = 0; local_index_y < map_length_grid; local_index_y++)
    {
      auto idx = local_index_x + local_index_y * map_length_grid;
      if (occupancy_grid_inflate_[idx] == 0)
      {
        occupancy_grid_neg[idx] = 1;
      }
      else if (occupancy_grid_inflate_[idx] == 1)
      {
        occupancy_grid_neg[idx] = 0;
      }
      else
      {
        ROS_ERROR("what?");
      }
    }
  }
  for (int local_index_x = 0; local_index_x < map_length_grid; local_index_x++)
  {
    fillESDF(
        [&](int local_index_y) {
          auto idx = local_index_x + local_index_y * map_length_grid;
          return occupancy_grid_neg[idx] == 1 ? 0 : std::numeric_limits<double>::max();
        },
        [&](int local_index_y, double val) {
          auto idx = local_index_x + local_index_y * map_length_grid;
          tmp_buffer1_[idx] = val;
        },
        0, map_length_grid - 1, 1);
  }
  for (int local_index_y = 0; local_index_y < map_length_grid; local_index_y++)
  {
    fillESDF(
        [&](int local_index_x) {
          auto idx = local_index_x + local_index_y * map_length_grid;
          return tmp_buffer1_[idx];
        },
        [&](int local_index_x, double val) {
          auto idx = local_index_x + local_index_y * map_length_grid;
          distance_buffer_neg_[idx] = map_resolution_ * std::sqrt(val);
          //  min(mp_.resolution_ * std::sqrt(val),
          //      md_.distance_buffer_[toAddress(x, y, z)]);
        },
        0, map_length_grid - 1, 0);
  }
  /* ========== combine pos and neg DT ========== */
  for (int local_index_x = 0; local_index_x < map_length_grid; local_index_x++)
  {
    for (int local_index_y = 0; local_index_y < map_length_grid; local_index_y++)
    {
      auto idx = local_index_x + local_index_y * map_length_grid;
      distance_buffer_all_[idx] = distance_buffer_[idx];

      if (distance_buffer_neg_[idx] > 0.0)
        distance_buffer_all_[idx] += (-distance_buffer_neg_[idx] + map_resolution_);
    }
  }
}

template <typename F_get_val, typename F_set_val>
void VehicleCentriMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim)
{
  int size;
  switch (dim)
  {
    case 0:
      size = map_length_grid;
      break;

    case 1:
      size = map_length_grid;
      break;
  }

  int v[size];
  double z[size + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++)
  {
    k++;
    double s;

    do
    {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) /
          (2 * q - 2 * v[k]);  //两占据栅格对应的距离抛物线的唯一交点表达式
    } while (s <= z[k]);

    k++;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++)
  {
    while (z[k + 1] < q)
    {
      k++;
    }
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

inline void VehicleCentriMap::boundIndex(int& id)
{
  int id1;
  id1 = max(min(id, map_length_grid - 1), 0);
  id = id1;

}  //限制索引不能超出地图范围

void VehicleCentriMap::publishESDF()
{
  if (esdf_pub_.getNumSubscribers() == 0)
    return;

  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  for (int local_index_x = 0; local_index_x < map_length_grid; local_index_x++)
  {
    for (int local_index_y = 0; local_index_y < map_length_grid; local_index_y++)
    {
      auto id = local_index_x + local_index_y * map_length_grid;

      dist = distance_buffer_[id];
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);

      pt.x = (local_index_x + 0.5) * map_resolution_ + map_origin_x;
      pt.y = (local_index_y + 0.5) * map_resolution_ + map_origin_y;
      pt.z = -0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);  //根据与障碍物之间的距离大小设置强度

      cloud.push_back(pt);
    }
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = "odom";
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);
}

void VehicleCentriMap::publishMap()
{
  if (map_inf_pub_.getNumSubscribers() == 0)
    return;
  inflated_cloud_to_pub.reset(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::PointXYZ pt;

  for (int local_index_x = 0; local_index_x < map_length_grid; local_index_x++)
  {
    for (int local_index_y = 0; local_index_y < map_length_grid; local_index_y++)
    {
      auto idx = local_index_x + local_index_y * map_length_grid;
      if (occupancy_grid_inflate_[idx] == 0)
        continue;

      pt.x = (local_index_x + 0.5) * map_resolution_ + map_origin_x;
      pt.y = (local_index_y + 0.5) * map_resolution_ + map_origin_y;
      pt.z = 0;
      inflated_cloud_to_pub->push_back(pt);
    }
  }

  inflated_cloud_to_pub->width = inflated_cloud_to_pub->points.size();
  inflated_cloud_to_pub->height = 1;
  inflated_cloud_to_pub->is_dense = true;
  inflated_cloud_to_pub->header.frame_id = "odom";

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*inflated_cloud_to_pub, cloud_msg);
  map_inf_pub_.publish(cloud_msg);
}

inline bool VehicleCentriMap::isInMap(const Eigen::Vector2d& pos)
{
  double dx = pos.x() - map_origin_x;
  double dy = pos.y() - map_origin_y;
  return (dx > 0 && dx < map_length && dy > 0 && dy < map_length);
}

void VehicleCentriMap::evaluateEDTWithGrad(const Eigen::Vector2d& pos, double& dist, Eigen::Vector3d& grad)
{
  if (!isInMap(pos))
  {
    cout << "pos invalid for interpolation." << endl;
  }

  Eigen::Vector2d diff;
  Eigen::Vector2d sur_pts[2][2];
  getSurroundPts(pos, sur_pts, diff);

  double dists[2][2];
  getSurroundDistance(sur_pts, dists);
  interpolateBilinear(dists, diff, dist, grad);
}

double VehicleCentriMap::evaluateCoarseEDT(const Eigen::Vector2d& pos)
{
  int ix = floor((pos.x() - map_origin_x) * map_resolution_inv_);
  boundIndex(ix);
  int iy = floor((pos.y() - map_origin_y) * map_resolution_inv_);
  boundIndex(iy);
  int idx = ix + iy * map_length_grid;
  return distance_buffer_[idx];
}

void VehicleCentriMap::interpolateBilinear(double values[2][2], const Eigen::Vector2d& diff, double& value,
                                           Eigen::Vector3d& grad)
{
  //沿着x轴插值
  double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
  double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
  //沿着y轴插值
  value = (1 - diff[1]) * v0 + diff[1] * v1;
  //计算梯度值
  grad[2] = 0;
  grad[1] = (v1 - v0) * map_resolution_inv_;
  grad[0] =
      ((1 - diff[1]) * (values[1][0] - values[0][0]) + diff[1] * (values[1][1] - values[0][1])) * map_resolution_inv_;
}

inline double VehicleCentriMap::getDistance(const Eigen::Vector2d& pos)
{
  int ix = floor((pos.x() - map_origin_x) * map_resolution_inv_);
  boundIndex(ix);
  int iy = floor((pos.y() - map_origin_y) * map_resolution_inv_);
  boundIndex(iy);
  int idx = ix + iy * map_length_grid;
  return distance_buffer_all_[idx];
}  //获取当前点与最近障碍物之间的距离

void VehicleCentriMap::getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2])
{
  for (int x = 0; x < 2; x++)
  {
    for (int y = 0; y < 2; y++)
    {
      dists[x][y] = getDistance(pts[x][y]);
    }
  }
}

void VehicleCentriMap::getSurroundPts(const Eigen::Vector2d& pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d& diff)
{
  Eigen::Vector2d pos_m = pos - 0.5 * map_resolution_ * Eigen::Vector2d::Ones();
  Eigen::Vector2i idx;
  Eigen::Vector2d idx_pos;

  idx = pos2LocalIndex(pos_m);
  localIndex2Pos(idx, idx_pos);
  diff = (pos - idx_pos) * map_resolution_inv_;

  for (int x = 0; x < 2; x++)
  {
    for (int y = 0; y < 2; y++)
    {
      Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
      Eigen::Vector2d current_pos;
      localIndex2Pos(current_idx, current_pos);
      pts[x][y] = current_pos;
    }
  }
}

inline Eigen::Vector2i VehicleCentriMap::pos2LocalIndex(const Eigen::Vector2d& pos)
{
  int ix = floor((pos[0] - map_origin_x) * map_resolution_inv_);
  int iy = floor((pos[1] - map_origin_y) * map_resolution_inv_);
  boundIndex(ix);
  boundIndex(iy);
  return Eigen::Vector2i(ix, iy);
}

inline void VehicleCentriMap::localIndex2Pos(const Eigen::Vector2i& id, Eigen::Vector2d& pos)
{
  pos(0) = (id(0) + 0.5) * map_resolution_ + map_origin_x;
  pos(1) = (id(1) + 0.5) * map_resolution_ + map_origin_y;
}

int VehicleCentriMap::getInflateOccupancy(Eigen::Vector2d pos)
{
  if (!isInMap(pos))
    return -1;
  int ix = floor((pos[0] - map_origin_x) * map_resolution_inv_);
  int iy = floor((pos[1] - map_origin_y) * map_resolution_inv_);
  boundIndex(ix);
  boundIndex(iy);
  int id = ix + iy * map_length_grid;
  return int(occupancy_grid_inflate_[id]);
}

void VehicleCentriMap::setOrigin(Eigen::Vector3d& ori, Eigen::Vector3d& size)
{
  ori = Eigen::Vector3d(map_origin_x + map_length / 2, map_origin_y + map_length / 2, 0);
  size = Eigen::Vector3d(map_length, map_length, map_resolution_);
}
