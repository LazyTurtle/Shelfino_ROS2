#include <cstdio>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "roadmap_interfaces/srv/path_service.hpp"
#include "visualization_msgs/msg/marker.hpp"  

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
using std::placeholders::_1;
using std::placeholders::_2;

#include "boost/polygon/voronoi.hpp"
#include "boost/polygon/polygon.hpp"
#include "boost/polygon/point_data.hpp"
#include "boost/polygon/segment_data.hpp"

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;

typedef boost::polygon::point_data<float> BoostPoint;
typedef boost::polygon::segment_data<float> BoostSegment;

using namespace std::chrono_literals;

class RoadmapManager : public rclcpp::Node
{
  public:
    RoadmapManager()
    : Node("RoadmapManager"){
      border_subscriber = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
      "map_borders", 1, std::bind(&RoadmapManager::set_borders, this, _1));
      
      obstacles_subscriber = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      "obstacles", 1, std::bind(&RoadmapManager::set_obstacles, this, _1));

      path_service = this->create_service<roadmap_interfaces::srv::PathService>(
        "compute_path", std::bind(&RoadmapManager::compute_path, this, _1, _2));

      test_service = this->create_service<std_srvs::srv::Empty>(
        "test_service", std::bind(&RoadmapManager::test, this, _1, _2));

      diagram_publisher = this->create_publisher<visualization_msgs::msg::Marker>(
        "voronoi_diagram", 1);

      auto interval = 1000ms;
      timer_ = this->create_wall_timer(interval, std::bind(&RoadmapManager::publish_diagram, this));

      
    }

  private:

    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr border_subscriber;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_subscriber;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr diagram_publisher;
    rclcpp::TimerBase::SharedPtr timer_;


    rclcpp::Service<roadmap_interfaces::srv::PathService>::SharedPtr path_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr test_service;

    std::shared_ptr<geometry_msgs::msg::PolygonStamped> borders_ptr;
    std::shared_ptr<obstacles_msgs::msg::ObstacleArrayMsg> obstacles_ptr;

    voronoi_diagram<double> vd;
    bool bIsDiagramReady = false;

    void log(std::string log_str){
      RCLCPP_INFO(this->get_logger(), log_str);
    }
    void log(std::ostringstream log_str){
      log(log_str.str());
    }

    void set_borders(const geometry_msgs::msg::PolygonStamped::SharedPtr msg){
      borders_ptr = msg;
    }

    void set_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg){
      obstacles_ptr = msg;
    }

    void test(
      const std::shared_ptr<std_srvs::srv::Empty_Request> request,
      std::shared_ptr<std_srvs::srv::Empty_Response> response){

      std::vector<BoostSegment> segments;
      std::vector<BoostPoint> points;

      auto border = borders_ptr->polygon;
      {
        for(int i = 0; i<border.points.size()-1; i++){
          BoostPoint a(border.points[i].x,border.points[i].y);
          BoostPoint b(border.points[i+1].x,border.points[i+1].y);
          segments.push_back(BoostSegment(a, b));
          points.push_back(a);
        }
        BoostPoint a(border.points.back().x,border.points.back().y);
        BoostPoint b(border.points[0].x,border.points[0].y);
        segments.push_back(BoostSegment(a, b));
        points.push_back(a);
      }

      auto obstacles = obstacles_ptr->obstacles;
      {
        for(auto& obs:obstacles){
          for(int i = 0; i<obs.polygon.points.size()-1; i++){
            BoostPoint a(obs.polygon.points[i].x, obs.polygon.points[i].y);
            BoostPoint b(obs.polygon.points[i+1].x, obs.polygon.points[i+1].y);
            segments.push_back(BoostSegment(a, b));
            points.push_back(a);
          }
          BoostPoint a(obs.polygon.points.back().x, obs.polygon.points.back().y);
          BoostPoint b(obs.polygon.points[0].x, obs.polygon.points[0].y);
          segments.push_back(BoostSegment(a, b));
          points.push_back(a);
        }
      }

      vd.clear();
      boost::polygon::construct_voronoi(
        points.begin(), points.end(), segments.begin(), segments.end(), &vd);
      bIsDiagramReady = true;
      
    }

    void compute_path(
      const std::shared_ptr<roadmap_interfaces::srv::PathService_Request> request,
      std::shared_ptr<roadmap_interfaces::srv::PathService_Response> response){
        response->result = false;
    }

    void publish_diagram(){
      if(!bIsDiagramReady)
        return;
        
      visualization_msgs::msg::Marker marker;

      marker.header.stamp = this->now();
      marker.header.frame_id = "map";
      marker.id = 0;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      int result = 0;
      for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin();
          it != vd.edges().end(); ++it) {
        if (it->is_primary()){
          ++result;
          geometry_msgs::msg::Point a, b;
          a.x = it->vertex0()->x();
          a.y = it->vertex0()->y();
          marker.points.push_back(a);
          b.x = it->vertex1()->x();
          b.y = it->vertex1()->y();
          marker.points.push_back(b);

        }
      }
      log(std::to_string(result));

      diagram_publisher->publish(marker);
    }

    
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoadmapManager>());
  rclcpp::shutdown();
  return 0;
}
