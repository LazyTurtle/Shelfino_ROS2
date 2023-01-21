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
        obstacle_array_pub = this->create_publisher<obstacles_msgs::msg::ObstacleArrayMsg>("obstacles", 1);

        obstacle_array = create_obstacles();

        for(int i = 0; i<obstacle_array.obstacles.size(); i++){
          std::ostringstream s;
          s <<"obs"<<(i+1);
          pubs.push_back(this->create_publisher<geometry_msgs::msg::PolygonStamped>(s.str(), 1));
        }

        auto interval = 1000ms;
        timer_ = this->create_wall_timer(interval, std::bind(&ObstaclesPublisher::publish_obstacles, this));
        RCLCPP_INFO_STREAM(this->get_logger(), "Ready to publish obstacles.");
    }

  
  private:
    
    rclcpp::Publisher<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_array_pub;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr> pubs;

    rclcpp::TimerBase::SharedPtr timer_;
    obstacles_msgs::msg::ObstacleArrayMsg obstacle_array;

    void publish_obstacles(){
      std_msgs::msg::Header new_header;
      new_header.frame_id = "map";
      new_header.stamp = this->now();

      obstacle_array.header = new_header;
      obstacle_array_pub->publish(obstacle_array);

      for(int i = 0; i < obstacle_array.obstacles.size(); i++){
        geometry_msgs::msg::PolygonStamped poly;
        poly.header = new_header;
        poly.polygon = obstacle_array.obstacles[i].polygon;
        pubs[i]->publish(poly);
      }
    }

    obstacles_msgs::msg::ObstacleArrayMsg create_obstacles(){

      std_msgs::msg::Header hh;

      hh.stamp = this->now();
      hh.frame_id = "map";

      obstacles_msgs::msg::ObstacleArrayMsg msg;
      std::vector<obstacles_msgs::msg::ObstacleMsg> obs_temp;

      obs_temp.push_back(build_square_obs(0.5,0.5,1.0));
      obs_temp.push_back(build_square_obs(-1.5,-1.5,1.0));
      obs_temp.push_back(build_square_obs(2,2,0.6));
      obs_temp.push_back(build_square_obs(-2,3,1.4));
      obs_temp.push_back(build_square_obs(3,-3.5,1.2));

      msg.header = hh;
      msg.obstacles = obs_temp;

      return msg;
    }

    obstacles_msgs::msg::ObstacleMsg build_square_obs(float x, float y, float l){
      obstacles_msgs::msg::ObstacleMsg obs;
      geometry_msgs::msg::Polygon pol;
      geometry_msgs::msg::Point32 point;
      std::vector<geometry_msgs::msg::Point32> points_temp;
      point.x = x-l/2;
      point.y = y+l/2;
      point.z = 0;
      points_temp.push_back(point);
      point.x = x+l/2;
      point.y = y+l/2;
      point.z = 0;
      points_temp.push_back(point);
      point.x = x+l/2;
      point.y = y-l/2;
      point.z = 0;
      points_temp.push_back(point);
      point.x = x-l/2;
      point.y = y-l/2;
      point.z = 0;
      points_temp.push_back(point);

      pol.points = points_temp;
      obs.polygon = pol;

      return obs;
    }
    


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstaclesPublisher>());
  rclcpp::shutdown();
  return 0;
}