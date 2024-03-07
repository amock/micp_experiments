#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <micp_mulran2/dataset_loading.h>

#include <rmw/qos_profiles.h>

using namespace micp_mulran2;
using namespace std::chrono_literals;

bool enable_visualization = true;

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr  pub_pcl;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr         pub_imu;
rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr   pub_gps;

void imuCB(size_t stamp_ns, const sensor_msgs::msg::Imu& imu)
{
  if(enable_visualization)
  {
    pub_imu->publish(imu);
  }
}

void gpsCB(size_t stamp_ns, const sensor_msgs::msg::NavSatFix& gps)
{
  if(enable_visualization)
  {
    pub_gps->publish(gps);
  }
}

void cloudCB(size_t stamp_ns, const sensor_msgs::msg::PointCloud& cloud)
{
  if(enable_visualization)
  {
    pub_pcl->publish(cloud);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::cout << "TEST" << std::endl;

  MulranDataset ds("/media/amock/OricoAlex/uni/datasets/mulran/raw/KAIST/KAIST01");

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos_sensor_data = rclcpp::QoS(
    rclcpp::QoSInitialization(
      qos_profile.history,
      qos_profile.depth
    ),
    qos_profile);

  node = std::make_shared<rclcpp::Node>("micp_mulran2_node");
  pub_pcl = node->create_publisher<sensor_msgs::msg::PointCloud>("cloud", qos_sensor_data);
  pub_imu = node->create_publisher<sensor_msgs::msg::Imu>("imu", qos_sensor_data);
  pub_gps = node->create_publisher<sensor_msgs::msg::NavSatFix>("gps", qos_sensor_data);


  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  for(auto msg = ds.next_message(); msg.first > 0; msg = ds.next_message())
  {
    size_t stamp = msg.first;
    if(msg.second.type() == typeid(sensor_msgs::msg::PointCloud))
    {
      std::cout << stamp << ": PCL" << std::endl; 
      cloudCB(msg.first, std::any_cast<sensor_msgs::msg::PointCloud>(msg.second));
    } else if(msg.second.type() == typeid(sensor_msgs::msg::Imu)) {
      std::cout << stamp << ": IMU" << std::endl;
      imuCB(msg.first, std::any_cast<sensor_msgs::msg::Imu>(msg.second));
    } else if(msg.second.type() == typeid(sensor_msgs::msg::NavSatFix)) {
      std::cout << stamp << ": GPS" << std::endl;
      gpsCB(msg.first, std::any_cast<sensor_msgs::msg::NavSatFix>(msg.second));
    }
    node->get_clock()->sleep_for(1ms);
    executor.spin_some();
  }

  return 0;
}