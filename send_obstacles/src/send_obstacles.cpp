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
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;


class ObstaclesPublisher : public rclcpp::Node
{
  public:
    ObstaclesPublisher()
    : Node("obstacles_sender")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        obstacle_array_pub = this->create_publisher<obstacles_msgs::msg::ObstacleArrayMsg>("obstacles", qos);
        pub1 = this->create_publisher<geometry_msgs::msg::PolygonStamped>("obs1", qos);
        pub2 = this->create_publisher<geometry_msgs::msg::PolygonStamped>("obs2", qos);

        obstacle_array = create_obstacles();

        auto interval = 1000ms;
        timer_ = this->create_wall_timer(interval, std::bind(&ObstaclesPublisher::publish_obstacles, this));

    }

  
  private:
    
    rclcpp::Publisher<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_array_pub;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub1;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub2;


    rclcpp::TimerBase::SharedPtr timer_;
    obstacles_msgs::msg::ObstacleArrayMsg obstacle_array;

    void publish_obstacles(){
      std_msgs::msg::Header new_header;
      new_header.frame_id = "map";
      new_header.stamp = this->now();

      obstacle_array.header = new_header;
      obstacle_array_pub->publish(obstacle_array);

      std::vector<geometry_msgs::msg::PolygonStamped> obstacles;
      for(auto obs_msg:obstacle_array.obstacles){
        geometry_msgs::msg::PolygonStamped poly;
        poly.header = new_header;
        poly.polygon = obs_msg.polygon;
        obstacles.push_back(poly);
      }
      pub1->publish(obstacles[0]);
      pub2->publish(obstacles[1]);

    }

    obstacles_msgs::msg::ObstacleArrayMsg create_obstacles(){

      std_msgs::msg::Header hh;

      hh.stamp = this->now();
      hh.frame_id = "map";

      obstacles_msgs::msg::ObstacleArrayMsg msg;
      obstacles_msgs::msg::ObstacleMsg obs;
      std::vector<obstacles_msgs::msg::ObstacleMsg> obs_temp;
      geometry_msgs::msg::Polygon pol;
      geometry_msgs::msg::Point32 point;

      geometry_msgs::msg::PolygonStamped pol1;
      geometry_msgs::msg::PolygonStamped pol2;

      pol1.header = hh;
      pol2.header = hh;

      // First square obstacle
      {
        std::vector<geometry_msgs::msg::Point32> points_temp;
        point.x = 0;
        point.y = 0;
        point.z = 0;
        points_temp.push_back(point);
        point.x = 0;
        point.y = 1;
        point.z = 0;
        points_temp.push_back(point);
        point.x = 1;
        point.y = 1;
        point.z = 0;
        points_temp.push_back(point);
        point.x = 1;
        point.y = 0;
        point.z = 0;
        points_temp.push_back(point);
        pol.points = points_temp;
        obs.polygon = pol;
        obs_temp.push_back(obs);
        pol1.polygon = pol;
      }

      // second square obstacle
      {
        std::vector<geometry_msgs::msg::Point32> points_temp;
        point.x = -1;
        point.y = -1;
        point.z = 0;
        points_temp.push_back(point);
        point.x = -1;
        point.y = -2;
        point.z = 0;
        points_temp.push_back(point);
        point.x = -2;
        point.y = -2;
        point.z = 0;
        points_temp.push_back(point);
        point.x = -2;
        point.y = -1;
        point.z = 0;
        points_temp.push_back(point);
        pol.points = points_temp;
        obs.polygon = pol;
        obs_temp.push_back(obs);
        pol2.polygon = pol;
      }

      msg.obstacles = obs_temp;

      return msg;
    }


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstaclesPublisher>());
  rclcpp::shutdown();
  return 0;
}