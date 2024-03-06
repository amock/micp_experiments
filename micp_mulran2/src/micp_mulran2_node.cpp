#include <rclcpp/rclcpp.hpp>

#include <micp_mulran2/dataset_loading.h>


using namespace micp_mulran2;


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::cout << "TEST" << std::endl;

  MulranDataset ds("/media/amock/OricoAlex/uni/datasets/mulran/raw/KAIST/KAIST01");


  for(auto msg = ds.next_message(); msg.first > 0; msg = ds.next_message())
  {
    size_t stamp = msg.first;
    if(msg.second.type() == typeid(sensor_msgs::msg::PointCloud))
    {
      std::cout << stamp << ": PCL" << std::endl; 
    } else if(msg.second.type() == typeid(sensor_msgs::msg::Imu)) {
      std::cout << stamp << ": IMU" << std::endl;
    } else if(msg.second.type() == typeid(sensor_msgs::msg::NavSatFix)) {
      std::cout << stamp << ": GPS" << std::endl;
    }
  }



  return 0;
}