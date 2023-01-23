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
#include "geometry_msgs/msg/pose_array.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
using std::placeholders::_1;
using std::placeholders::_2;

#include "boost/polygon/voronoi.hpp"
#include "boost/polygon/polygon.hpp"
#include "boost/polygon/point_data.hpp"
#include "boost/polygon/segment_data.hpp"
#include "voronoi_visualization_utils.hpp"

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;

typedef boost::polygon::point_data<double> BoostPoint;
typedef boost::polygon::segment_data<double> BoostSegment;

using namespace std::chrono_literals;

class RoadmapManager : public rclcpp::Node
{
  public:
    RoadmapManager()
    : Node("RoadmapManager"){
      const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

      border_subscriber = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
      "map_borders", qos, std::bind(&RoadmapManager::set_borders, this, _1));
      
      obstacles_subscriber = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      "obstacles", qos, std::bind(&RoadmapManager::set_obstacles, this, _1));

      gates_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "gate_position", qos, std::bind(&RoadmapManager::set_gates, this, _1));

      path_service = this->create_service<roadmap_interfaces::srv::PathService>(
        "compute_path", std::bind(&RoadmapManager::compute_path, this, _1, _2));

      test_service = this->create_service<std_srvs::srv::Empty>(
        "test_service", std::bind(&RoadmapManager::test, this, _1, _2));

      diagram_publisher = this->create_publisher<visualization_msgs::msg::Marker>(
        "voronoi_diagram", qos);

      auto interval = 1000ms;
      timer_ = this->create_wall_timer(interval, std::bind(&RoadmapManager::publish_diagram_marker, this));

      
    }

  private:

    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr border_subscriber;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gates_subscriber;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr diagram_publisher;

    rclcpp::TimerBase::SharedPtr timer_;


    rclcpp::Service<roadmap_interfaces::srv::PathService>::SharedPtr path_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr test_service;

    std::shared_ptr<geometry_msgs::msg::PolygonStamped> borders_msg;
    std::shared_ptr<obstacles_msgs::msg::ObstacleArrayMsg> obstacles_msg;
    std::shared_ptr<geometry_msgs::msg::PoseArray> gates_msg;

    visualization_msgs::msg::Marker vd_marker;    

    voronoi_diagram<double> vd;
    std::vector<BoostSegment> segments_data;
    std::vector<BoostPoint> points_data;

    int scale = 100;
    int discretization = 0.2 * scale;

    void log(std::string log_str){
      RCLCPP_INFO(this->get_logger(), log_str);
    }

    void set_borders(const geometry_msgs::msg::PolygonStamped::SharedPtr msg){
      borders_msg = msg;
    }

    void set_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg){
      obstacles_msg = msg;
    }

    void set_gates(const geometry_msgs::msg::PoseArray::SharedPtr msg){
      gates_msg = msg;
    }

    void test(
      const std::shared_ptr<std_srvs::srv::Empty_Request> request,
      std::shared_ptr<std_srvs::srv::Empty_Response> response){
      update_voronoi_diagram();
    }

    void update_voronoi_diagram(){
      
      if(!borders_msg || !obstacles_msg){
        log("Pointers not ready.");
        return;
      }

      segments_data.clear();
      points_data.clear();

      add_polygon_to_boost_segments(borders_msg->polygon, segments_data);

      std::vector<obstacles_msgs::msg::ObstacleMsg> obstacles = obstacles_msg->obstacles;
      for(auto& obs:obstacles){
        add_polygon_to_boost_segments(obs.polygon, segments_data);
      }

      vd.clear();
      boost::polygon::construct_voronoi(
        points_data.begin(), points_data.end(),
        segments_data.begin(), segments_data.end(), &vd);
      
      log("Updated voronoi diagram.");

      update_diagram_marker();

    }

    void add_polygon_to_boost_segments(
      const geometry_msgs::msg::Polygon& poly, std::vector<BoostSegment>& segments){

      for(int i = 0; i<poly.points.size()-1; i++){
        BoostPoint a = g2b_p(poly.points[i], scale);
        BoostPoint b = g2b_p(poly.points[i+1], scale);
        BoostSegment segment(a, b);
        segments.push_back(segment);
      }
      BoostPoint a = g2b_p(poly.points.back(), scale);
      BoostPoint b = g2b_p(poly.points[0], scale);
      BoostSegment segment(a, b);

      segments.push_back(segment);
    }

    // geometry to boost, point
    BoostPoint g2b_p(const geometry_msgs::msg::Point32 geo_point, const int scale = 1){
      double x = geo_point.x * scale;
      double y = geo_point.y * scale;
      BoostPoint p(x, y);
      return p;
    }

    void compute_path(
      const std::shared_ptr<roadmap_interfaces::srv::PathService_Request> request,
      std::shared_ptr<roadmap_interfaces::srv::PathService_Response> response){
        response->result = false;
    }

    void publish_diagram_marker(){
      diagram_publisher->publish(vd_marker);
    }

    void update_diagram_marker(){        
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
      for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it) {
        
        if (it->is_primary() && it->is_finite()){
          ++result;
          if(it->is_linear()){
            geometry_msgs::msg::Point a, b;
            auto tx = it->vertex0()->x();
            auto ty = it->vertex0()->y();
            a.x = tx/scale;
            a.y = ty/scale;
            marker.points.push_back(a);
            tx = it->vertex1()->x();
            ty = it->vertex1()->y();
            b.x = tx/scale;
            b.y = ty/scale;
            marker.points.push_back(b);
          }else{
            // is curved
            std::vector<BoostPoint> points;
            BoostPoint vertex0(it->vertex0()->x(), it->vertex0()->y());
            points.push_back(vertex0);
            BoostPoint vertex1(it->vertex1()->x(), it->vertex1()->y());
            points.push_back(vertex1);

            boost::polygon::voronoi_edge<double> edge = *it;


            BoostPoint point = edge.cell()->contains_point() ?
              retrieve_point(*edge.cell()) :
              retrieve_point(*edge.twin()->cell());
            BoostSegment segment = edge.cell()->contains_point() ?
              retrieve_segment(*edge.twin()->cell()) :
              retrieve_segment(*edge.cell());
            
            boost::polygon::voronoi_visual_utils<double>::discretize(
            point, segment, discretization, &points);

            for(int i = 0; i<points.size()-1; i++){
              auto a = points[i];
              auto b = points[i+1];
              geometry_msgs::msg::Point ma, mb;
              ma.x = a.x()/scale;
              ma.y = a.y()/scale;
              mb.x = b.x()/scale;
              mb.y = b.y()/scale;
              marker.points.push_back(ma);
              marker.points.push_back(mb);
            }
          }

        }
      }
      vd_marker = marker;
      std::ostringstream s;
      s << "Updated marker.";
      log(s.str());
    }

    BoostPoint retrieve_point(const boost::polygon::voronoi_cell<double>& cell) {
      auto index = cell.source_index();
      auto category = cell.source_category();
      if (category == boost::polygon::SOURCE_CATEGORY_SINGLE_POINT) {
        return points_data[index];
      }
      index -= points_data.size();
      if (category == boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT) {
        return low(segments_data[index]);
      } else {
        return high(segments_data[index]);
      }
    }

    BoostSegment retrieve_segment(const boost::polygon::voronoi_cell<double>& cell) {
      auto index = cell.source_index() - points_data.size();
      return segments_data[index];
    }
    
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoadmapManager>());
  rclcpp::shutdown();
  return 0;
}
