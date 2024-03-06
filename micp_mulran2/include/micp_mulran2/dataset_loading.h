#ifndef MICP_EXP_DATASET_LOADING_H
#define MICP_EXP_DATASET_LOADING_H

#include <string>
#include <iostream>
#include <fstream>
#include <utility>
#include <variant>
#include <any>

#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include <rmagine/types/sensor_models.h>

#include <boost/filesystem.hpp>

namespace micp_mulran2
{

using SensorData = std::variant<
  sensor_msgs::msg::PointCloud,
  sensor_msgs::msg::Imu
>;

class MulranDataset
{
public:
  MulranDataset(std::string mulran_root);
  std::pair<size_t, std::any> next_message();

// protected:
  std::pair<size_t, sensor_msgs::msg::PointCloud> next_ouster();
  std::pair<size_t, sensor_msgs::msg::Imu>        next_imu();
  std::pair<size_t, sensor_msgs::msg::NavSatFix>  next_gps();

  std::vector<std::pair<size_t, std::any> > m_buffer;

  std::ifstream m_imu_file;
  std::ifstream m_gps_file;
  std::vector<size_t> m_ouster_stamps;
  size_t m_ouster_id = 0;

  boost::filesystem::path m_ouster_root;
};

} // namespace micp_mulran2

#endif // MICP_EXP_DATASET_LOADING_H