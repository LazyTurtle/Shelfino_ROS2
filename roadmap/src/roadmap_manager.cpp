#include <cstdio>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

using std::placeholders::_1;

class RoadmapManager : public rclcpp::Node
{
  public:
    RoadmapManager()
    : Node("RoadmapManager"){
      subscription_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
      "borders", 10, std::bind(&RoadmapManager::topic_callback, this, _1));
    }

  private:
    void topic_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr borders) const{
      RCLCPP_INFO(this->get_logger(), "I heard it!");
      std::ostringstream s;
      s << borders.get()->header.stamp.sec;
      RCLCPP_INFO(this->get_logger(), s.str());
    }
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoadmapManager>());
  rclcpp::shutdown();
  return 0;
}
