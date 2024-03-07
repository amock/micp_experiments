#include <micp_mulran2/dataset_loading.h>


#include <rclcpp/rclcpp.hpp>

#include <iostream>


#include <boost/algorithm/string.hpp>
#include <string>
#include <chrono>


namespace bfs = boost::filesystem;

namespace micp_mulran2
{

MulranDataset::MulranDataset(std::string mulran_root)
{
  bfs::path p{mulran_root};
  bfs::path ouster_p = p / "sensor_data" / "Ouster";
  bfs::path gps_p = p / "sensor_data" / "gps.csv";
  bfs::path imu_p = p / "sensor_data" / "xsens_imu.csv";

  m_ouster_root = ouster_p;

  m_imu_file.open(imu_p.string());
  if(!m_imu_file.is_open())
  {
    std::cout << "ERROR opening: " << imu_p << std::endl;
  }

  m_gps_file.open(gps_p.string());
  if(!m_gps_file.is_open())
  {
    std::cout << "ERROR opening: " << gps_p << std::endl;
  }

  m_ouster_stamps.clear();
  for(auto&& x : bfs::directory_iterator(ouster_p))
  {
    auto p = x.path();
    if(p.extension() == ".bin")
    {
      size_t stamp = std::stol(p.stem().string());
      m_ouster_stamps.push_back(stamp);
    }
  }

  // sort stamps in ascending order
  std::sort(m_ouster_stamps.begin(), m_ouster_stamps.end());

  m_buffer.resize(3);
  m_buffer[0] = next_ouster();
  m_buffer[1] = next_imu();
  m_buffer[2] = next_gps();
}

std::pair<size_t, std::any> MulranDataset::next_message()
{
  std::pair<size_t, std::any> ret;

  size_t min_id = 100000;
  size_t min_stamp = std::numeric_limits<size_t>::max();
  
  for(size_t i=0; i<m_buffer.size(); i++)
  {
    if(m_buffer[i].first < min_stamp)
    {
      min_id = i;
      min_stamp = m_buffer[i].first;
    }
  }

  // return earliest message and replace with new one
  ret = m_buffer[min_id];

  if(min_id == 0)
  {
    m_buffer[0] = next_ouster();
  } else if(min_id == 1) {
    m_buffer[1] = next_imu();
  } else if(min_id == 2) {
    m_buffer[2] = next_gps();
  } else {
    std::cout << "END DETECTED" << std::endl;
    // end
    ret.first = 0;
  }

  return ret;
}

std::pair<size_t, sensor_msgs::msg::PointCloud> MulranDataset::next_ouster()
{
  std::pair<size_t, sensor_msgs::msg::PointCloud> ret;

  if(m_ouster_id < m_ouster_stamps.size())
  {
    size_t stamp = m_ouster_stamps[m_ouster_id];
    bfs::path ouster_filename = m_ouster_root / (std::to_string(stamp) + ".bin");

    ret.first = stamp;
    ret.second.header.stamp = rclcpp::Time(ret.first);
    ret.second.header.frame_id = "ouster";

    // TODO: load file
    const size_t n_points = 65536;
    ret.second.points.resize(n_points);
    ret.second.channels.resize(1);
    ret.second.channels[0].name = "intensity";
    ret.second.channels[0].values.resize(n_points);
    
    float buf[n_points * 4];

    std::ifstream in(ouster_filename.string(), std::ios_base::binary);
    if(!in.read((char*)buf, 4 * n_points * sizeof(float)))
    {
      throw std::runtime_error("wrong");
    }

    for(size_t i=0; i<n_points; i++)
    {
      ret.second.points[i].x = buf[i * 4 + 0];
      ret.second.points[i].y = buf[i * 4 + 1];
      ret.second.points[i].z = buf[i * 4 + 2];
      ret.second.channels[0].values[i] = buf[i * 4 + 3];
    }

  } else {
    ret.first = 0;
  }

  m_ouster_id++;
  return ret;
}

std::pair<size_t, sensor_msgs::msg::Imu> MulranDataset::next_imu()
{
  std::pair<size_t, sensor_msgs::msg::Imu> ret;

  if(m_imu_file.peek() != EOF)
  {
    std::string imu_line;
    std::getline(m_imu_file, imu_line);
  
    // parse imu_line
    std::vector<std::string> imu_strs;
    boost::split(imu_strs, imu_line, boost::is_any_of(","));

    ret.first = stol(imu_strs[0]);
    ret.second.header.stamp = rclcpp::Time(ret.first);

    ret.second.orientation.x = stod(imu_strs[1]);
    ret.second.orientation.y = stod(imu_strs[2]);
    ret.second.orientation.z = stod(imu_strs[3]);
    ret.second.orientation.w = stod(imu_strs[4]);

    // imu_strs[5] -> euler x
    // imu_strs[6] -> euler y
    // imu_strs[7] -> euler z
    ret.second.angular_velocity.x = stod(imu_strs[ 8]);
    ret.second.angular_velocity.y = stod(imu_strs[ 9]);
    ret.second.angular_velocity.z = stod(imu_strs[10]);

    ret.second.linear_acceleration.x = stod(imu_strs[11]);
    ret.second.linear_acceleration.y = stod(imu_strs[12]);
    ret.second.linear_acceleration.z = stod(imu_strs[13]);

    // imu_strs[14] -> mag x
    // imu_strs[15] -> mag y
    // imu_strs[16] -> mag z
  } else {
    ret.first = 0;
  }

  return ret;
}

std::pair<size_t, sensor_msgs::msg::NavSatFix> MulranDataset::next_gps()
{
  std::pair<size_t, sensor_msgs::msg::NavSatFix> ret;

  if(m_gps_file.peek() != EOF)
  {
    std::string gps_line;
    std::getline(m_gps_file, gps_line);
    std::vector<std::string> gps_strs;
    boost::split(gps_strs, gps_line, boost::is_any_of(","));

    ret.first = stol(gps_strs[0]);
    ret.second.header.stamp = rclcpp::Time(ret.first);
    
    // lat, lon, alt
    ret.second.latitude = stod(gps_strs[1]);
    ret.second.longitude = stod(gps_strs[2]);
    ret.second.altitude = stod(gps_strs[3]);

    for(size_t i=0; i<9; i++)
    {
      ret.second.position_covariance[i] = stod(gps_strs[4 + i]);
    }

  } else {
    ret.first = 0;
  }

  return ret;
}


} // namespace micp_mulran2
