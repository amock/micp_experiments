#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/math/types.h>

namespace rm = rmagine;

using std::placeholders::_1;

class ClostestPointTestNode : public rclcpp::Node
{
public:
  ClostestPointTestNode()
  :rclcpp::Node("closest_point_test_node", rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true))
  {
    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", rclcpp::SensorDataQoS(), std::bind(&ClostestPointTestNode::pose_cb, this, _1));

    sub_point_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", rclcpp::SensorDataQoS(), std::bind(&ClostestPointTestNode::point_cb, this, _1));

    pub_corr_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "corr", rclcpp::SensorDataQoS());
  
    std::string map_filename = this->get_parameter("map_file").as_string();
    map_ = rm::import_embree_map(map_filename);
  }

  void point_cb(const geometry_msgs::msg::PointStamped& p) const
  {
    rm::Vector q;
    q.x = p.point.x;
    q.y = p.point.y;
    q.z = p.point.z;

    auto res = map_->closestPoint(q);

    float corr_scale = 0.05;

    visualization_msgs::msg::Marker corr;
    corr.header.frame_id = "map";
    corr.header.stamp = p.header.stamp;
    corr.id = 0;
    corr.type = visualization_msgs::msg::Marker::LINE_LIST;
    corr.action = visualization_msgs::msg::Marker::ADD;
    corr.pose.orientation.w = 1.0;
    corr.scale.x = corr_scale;
    corr.scale.y = corr_scale;
    corr.scale.z = corr_scale;

    std_msgs::msg::ColorRGBA red;
    red.r = 1.0;
    red.g = 0.0;
    red.b = 0.0;
    red.a = 1.0;

    geometry_msgs::msg::Point dros = p.point;
    geometry_msgs::msg::Point mros;
    mros.x = res.p.x;
    mros.y = res.p.y;
    mros.z = res.p.z;

    corr.points.push_back(dros);
    corr.points.push_back(mros);

    corr.colors.push_back(red);
    corr.colors.push_back(red);

    std::cout << "Publish marker" << std::endl;

    pub_corr_->publish(corr);
  }

  void pose_cb(const geometry_msgs::msg::PoseStamped& msg) const
  {
    std::cout << "Got pose" << std::endl;

    geometry_msgs::msg::PointStamped p;
    p.header = msg.header;
    p.point = msg.pose.position;

    point_cb(p);
  }

private:

  rm::EmbreeMapPtr map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_point_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_corr_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClostestPointTestNode>());
  rclcpp::shutdown();

  return 0;
}