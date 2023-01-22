#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

class GatesPublisher : public rclcpp::Node
{
  public:
    GatesPublisher()
    : Node("send_gates")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("gate_position", 10);

      auto interval = 1000ms;
      timer_ = this->create_wall_timer(interval, std::bind(&GatesPublisher::publish_gates, this));

      gates_poses = create_gates();

      RCLCPP_INFO_STREAM(this->get_logger(), "Ready to publish the gates.");
    }

  
  private:
    
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseArray gates_poses;

    void publish_gates(){
      std_msgs::msg::Header new_header;
      new_header.stamp = this->now();
      new_header.frame_id = "map";

      gates_poses.header = new_header;
      publisher_->publish(gates_poses);
    }

    geometry_msgs::msg::PoseArray create_gates(){
      
      geometry_msgs::msg::Pose pose;
      std::vector<geometry_msgs::msg::Pose> pose_array_temp;
      geometry_msgs::msg::PoseArray msg;

      pose.position.x = -5;
      pose.position.y = -2.5;
      pose.position.z = 0;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 0;
      pose_array_temp.push_back(pose);
      pose.position.x = -4;
      pose.position.y = 5;
      pose.position.z = 0;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 0;
      pose_array_temp.push_back(pose);
      msg.poses = pose_array_temp;

      return msg;
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GatesPublisher>());
  rclcpp::shutdown();
  return 0;
}