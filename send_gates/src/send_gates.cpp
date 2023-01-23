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


class GatesPublisher : public rclcpp::Node
{
  public:
    GatesPublisher()
    : Node("send_gates")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("gate_position", qos);

        std_msgs::msg::Header hh;
        geometry_msgs::msg::Pose pose;
        std::vector<geometry_msgs::msg::Pose> pose_array_temp;
        geometry_msgs::msg::PoseArray msg;

        hh.stamp = this->get_clock()->now();
        hh.frame_id = "map";

        msg.header = hh;

         pose.position.x = -5;
         pose.position.y = -2.5;
         //pose.position.x = -2.81;
         //pose.position.y = -4.46;
         pose.position.z = 0;
         pose.orientation.x = 0;
         pose.orientation.y = 0;
         pose.orientation.z = 0;
         pose.orientation.w = 0;
         pose_array_temp.push_back(pose);
         pose.position.x = -4;
         pose.position.y = 5;
         //pose.position.x = -1.36;
         //pose.position.y = -1.11;

        pose_array_temp.push_back(pose);
        msg.poses = pose_array_temp;

        while(1){
          publisher_->publish(msg);
          usleep(1000000);
        }
    }

  
  private:
    
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GatesPublisher>());
  rclcpp::shutdown();
  return 0;
}