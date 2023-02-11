#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>

#include "rclcpp/rclcpp.hpp"


#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include "std_srvs/srv/empty.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class PositionListener : public rclcpp::Node
{
  public:
    PositionListener()
    : Node("get_positions")
    {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

      tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("transform", qos);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&PositionListener::timer_callback, this));
      shutdown_srv = this->create_service<std_srvs::srv::Empty>(
        "shutdown", std::bind(&PositionListener::deactivate, this, _1, _2));
    }

  private:
    void timer_callback()
    {
      geometry_msgs::msg::TransformStamped t;

      try {
          rclcpp::Time now = this->get_clock()->now();
          t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero, 1s);
      } catch (const tf2::TransformException & ex) {
          // RCLCPP_INFO(this->get_logger(), "Could not transform map to base_link: %s", ex.what());
          // return;
      }
      publisher_->publish(t);
    }

    void deactivate(
      const std::shared_ptr<std_srvs::srv::Empty_Request> request,
      std::shared_ptr<std_srvs::srv::Empty_Response> response){

      timer_ = nullptr;
      publisher_= nullptr;
      std::string s = "Get position deactivated";
      rclcpp::shutdown(this->get_node_options().context(), s);
    }
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr shutdown_srv;
    size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionListener>());
  rclcpp::shutdown();
  return 0;
}