#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "dubins_planner_msgs/srv/multi_point_dubins_planning.hpp"


/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Path>("plan", 10);
      
    }

    void init(){
      auto client = this->create_client<dubins_planner_msgs::srv::MultiPointDubinsPlanning>("multi_points_dubins_calculator");
      auto request = std::make_shared<dubins_planner_msgs::srv::MultiPointDubinsPlanning::Request>();

      geometry_msgs::msg::Point p1;
      geometry_msgs::msg::Point p2;
      p2.x = 1;
      p2.y = -2;
      geometry_msgs::msg::Point p3;
      p3.x = 1;
      p3.y = 2;
      geometry_msgs::msg::Point p4;
      p4.x = 1.5;
      geometry_msgs::msg::Point p5;
      p5.x = 2.3;
      p5.y = 1;
      geometry_msgs::msg::Point p6;
      p6.x = 4;
      p6.y = -1;
      geometry_msgs::msg::Point p7;
      p7.x = 6;
      p7.y = 3;

      request->points.insert(request->points.end(),{p1,p2,p3,p4,p5,p6,p7});
      request->kmax = 3;
      request->angle = 1.5*3.141592;


      while (!client->wait_for_service(2s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      auto result = client->async_send_request(request);

      // Wait for the result.
      if (rclcpp::spin_until_future_complete(shared_from_this(),result) ==
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service dubins_calculator successfully called.");
        path = result.get()->path;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service dubins_calculator.");
      }
      timer_ = this->create_wall_timer
      (500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    size_t count_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path path;

    void timer_callback()
        {
          publisher_->publish(path);
        }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto pub = std::make_shared<MinimalPublisher>();
  pub->init();
  rclcpp::spin(pub);
  rclcpp::shutdown();
  
  return 0;
}