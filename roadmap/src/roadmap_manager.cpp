#include <cstdio>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
using std::placeholders::_1;

#include "boost/polygon/voronoi.hpp"
#include "boost/polygon/polygon.hpp"
#include "boost/polygon/point_data.hpp"
#include "boost/polygon/segment_data.hpp"

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;

typedef boost::polygon::point_data<float> BoostPoint;
typedef boost::polygon::segment_data<float> BoostSegment;

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
      borders_ptr = msg;
    }

    void set_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg){
      obstacles_ptr = msg;
    }

    void test(){
      std::vector<BoostSegment> segments;

      auto border = borders_ptr.get()->polygon;
      {
        for(int i = 0; i<border.points.size()-1; i++){
          BoostPoint a(border.points[i].x,border.points[i].y);
          BoostPoint b(border.points[i+1].x,border.points[i+1].y);
          segments.push_back(BoostSegment(a, b));
        }
        BoostPoint a(border.points.back().x,border.points.back().y);
        BoostPoint b(border.points[0].x,border.points[0].y);
        segments.push_back(BoostSegment(a, b));
      }

      auto obstacles = obstacles_ptr.get()->obstacles;
      {
        for(auto& obs:obstacles){
          for(int i = 0; i<obs.polygon.points.size()-1; i++){
            BoostPoint a(obs.polygon.points[i].x, obs.polygon.points[i].y);
            BoostPoint b(obs.polygon.points[i+1].x, obs.polygon.points[i+1].y);
            segments.push_back(BoostSegment(a, b));
          }
          BoostPoint a(obs.polygon.points.back().x, obs.polygon.points.back().y);
          BoostPoint b(obs.polygon.points[0].x, obs.polygon.points[0].y);
          segments.push_back(BoostSegment(a, b));
        }
      }
      voronoi_diagram<double> vd;
      boost::polygon::construct_voronoi(segments.begin(), segments.end(), &vd);

    }

    void build_vr_diagram(){
    }

    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr border_subscriber;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_subscriber;

    std::shared_ptr<geometry_msgs::msg::PolygonStamped> borders_ptr;
    std::shared_ptr<obstacles_msgs::msg::ObstacleArrayMsg> obstacles_ptr;

    
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoadmapManager>());
  rclcpp::shutdown();
  return 0;
}
