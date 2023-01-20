#include <cstdio>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"

using std::placeholders::_1;

class RoadmapManager : public rclcpp::Node
{
  public:
    RoadmapManager()
    : Node("RoadmapManager"){
      border_subscriber = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
      "borders", 1, std::bind(&RoadmapManager::set_borders, this, _1));
      
      obstacles_subscriber = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
        "obstacles", 1, std::bind(&RoadmapManager::set_obstacles, this, _1));
      
    }

  private:

    void set_borders(const geometry_msgs::msg::PolygonStamped::SharedPtr msg){
      borders = msg;
    }

    void set_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg){
      obstacles = msg;
    }

    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr border_subscriber;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_subscriber;

    std::shared_ptr<geometry_msgs::msg::PolygonStamped> borders;
    std::shared_ptr<obstacles_msgs::msg::ObstacleArrayMsg> obstacles;

    
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoadmapManager>());
  rclcpp::shutdown();
  return 0;
}
