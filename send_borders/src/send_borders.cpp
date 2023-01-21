#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>

#include "rclcpp/rclcpp.hpp"


#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

class BordersPublisher : public rclcpp::Node
{
  public:
    BordersPublisher()
    : Node("send_borders")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("map_borders", qos);

        auto interval = 1000ms;
        timer_ = this->create_wall_timer(interval, std::bind(&BordersPublisher::publish_borders, this));
        
        borders = get_borders();

    }

  
  private:
    
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PolygonStamped borders;

    void publish_borders(){
      borders.header.stamp = this->now();
      publisher_->publish(borders);
    }

    geometry_msgs::msg::PolygonStamped get_borders(){
      std_msgs::msg::Header hh;

      hh.stamp = this->get_clock()->now();
      hh.frame_id = "map";

      geometry_msgs::msg::Polygon pol;
      geometry_msgs::msg::Point32 point;

      geometry_msgs::msg::PolygonStamped pol1;

      pol1.header = hh;

      std::vector<geometry_msgs::msg::Point32> points_temp;
      point.x = -5;
      point.y = -5;
      point.z = 0;
      points_temp.push_back(point);
      point.x = -5;
      point.y = 5;
      point.z = 0;
      points_temp.push_back(point);
      point.x = 5;
      point.y = 5;
      point.z = 0;
      points_temp.push_back(point);
      point.x = 5;
      point.y = -5;
      point.z = 0;
      points_temp.push_back(point);
      pol.points = points_temp;
      pol1.polygon = pol;

      return pol1;
    }
    


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BordersPublisher>());
  rclcpp::shutdown();
  return 0;
}