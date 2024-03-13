#ifndef MICP_MULRAN2_VOXEL_FILTER_HPP
#define MICP_MULRAN2_VOXEL_FILTER_HPP

#include <sensor_msgs/msg/point_cloud.hpp>
#include <utility>
#include <limits>
#include <unordered_map>
#include <random>
#include <vector>



namespace micp_mulran2
{

struct VoxelCoord
{
  // could use int64_t but you are only allowed to write a int42_t. then we have more space
  int32_t x;
  int32_t y;
  int32_t z;


  bool operator==(const VoxelCoord &other) const
  { 
    return (x == other.x && y == other.y && z == other.z);
  }

  // with voxelsize 1cm the whole world fits in the hash map three times
  __int128 hash() const
  {
    return static_cast<__int128>(x) << 64 ^ static_cast<__int128>(y) << 32 ^ static_cast<__int128>(z);
  }
};

} // namespace micp_mulran2

namespace std
{
  template <>
  struct hash<micp_mulran2::VoxelCoord>
  {
    __int128 operator()(const micp_mulran2::VoxelCoord& c) const
    {
      return c.hash();
    }
  };

} // namespace std

namespace micp_mulran2
{

using HashMap = std::unordered_map<VoxelCoord, std::vector<geometry_msgs::msg::Point32> >;

class VoxelFilter
{
public:
  VoxelFilter(double voxel_size = 0.2)
  :m_voxel_size(voxel_size)
  {
    
  }

  inline
  void insert(const geometry_msgs::msg::Point32& p)
  {
    auto voxel = calc_voxel(p);
    m_hm[voxel].push_back(p);
  }

  inline
  void insert(const std::vector<geometry_msgs::msg::Point32>& points)
  {
    for(auto p : points)
    {
      insert(p);
    }
  }

  /**
   * filter_method:
   * 0: first point
  */
  inline
  std::vector<geometry_msgs::msg::Point32> get_filtered_points(const size_t filter_method = 0) const
  {
    std::vector<geometry_msgs::msg::Point32> points_out;
    std::mt19937 rng(42);
    
    for(const auto elem : m_hm)
    {
      if(filter_method == 0)
      {
        points_out.push_back(elem.second[0]);
      } else if(filter_method == 1) {
        size_t rand_id = std::uniform_int_distribution<size_t>(0, elem.second.size() - 1)(rng);
        points_out.push_back(elem.second[rand_id]);
      } else {
        throw std::runtime_error("FILTER METHOD NOT IMPLEMENTED");
      }
    }

    return points_out;
  }

private:

  inline
  VoxelCoord calc_voxel(const geometry_msgs::msg::Point32& p) const
  {
    VoxelCoord voxel;

    // this is wrong around zero
    voxel.x = static_cast<int32_t>(std::floor(p.x / m_voxel_size));
    voxel.y = static_cast<int32_t>(std::floor(p.y / m_voxel_size));
    voxel.z = static_cast<int32_t>(std::floor(p.z / m_voxel_size));

    return voxel;
  }

  double m_voxel_size;
  HashMap m_hm;

};

} // namespace micp_mulran2

#endif // MICP_MULRAN2_VOXEL_FILTER_HPP