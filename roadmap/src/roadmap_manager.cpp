#include <cstdio>
#include <memory>
#include <vector>
#include <set>
#include <algorithm>
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

class Node{

  static double distance(Node& a, Node& b){
    double x = a.x - b.x;
    double y = a.y - b.y;
    double d = std::sqrt(std::pow(x,2)+std::pow(y,2)); 
    return d; 
  }  

  public:

    double x, y;
    std::set<int> neighbours;
    Node(){}
    Node(double in_x, double in_y) : x(in_x), y(in_y){}


};

class Graph{

  public:

    std::vector<Node> nodes;

    Graph(){}
    Graph(std::vector<Node> init_nodes, std::vector<std::pair<int, int>> init_edges){
      nodes = init_nodes;
      clean_edges();
      add_edges(init_edges);
    }

    void add_node(double x, double y){
      Node n(x,y);
      nodes.push_back(n);
    }

    void clean_edges(){
      for(auto& node : nodes){
        node.neighbours.clear();
      }
    }

    void add_edges(const std::vector<std::pair<int, int>>& edges){
      for(auto& edge:edges)
        add_edge(edge);
    }

    void add_edge(std::pair<int, int> edge){
      add_edge(edge.first, edge.second);
    }

    void add_edge(int node_a, int node_b){
      nodes[node_a].neighbours.insert(node_b);
      nodes[node_b].neighbours.insert(node_a);
    }

};

class RoadmapManager : public rclcpp::Node
{
  public:
    RoadmapManager()
    : Node("RoadmapManager"){
      node_setup();
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
    Graph search_graph;
    std::vector<BoostSegment> segments_data;
    std::vector<BoostPoint> points_data;

    int scale = 100;
    int discretization = 0.2 * scale;

    void node_setup(){
      
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
      add_ids_to_vertices();
      build_search_graph();
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

    // voronoi to geometry, point
    geometry_msgs::msg::Point v2g_p(const boost::polygon::voronoi_vertex<double> vertex, const int scale = 1){
      geometry_msgs::msg::Point p;
      p.x = vertex.x() / scale;
      p.y = vertex.y() / scale;
      return p;
    }
    // boost to geometry, point
    geometry_msgs::msg::Point b2g_p(const BoostPoint point, const int scale = 1){
      geometry_msgs::msg::Point p;
      p.x = point.x() / scale;
      p.y = point.y() / scale;
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
      marker.scale.x = 0.05;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 0.5;
      marker.color.g = 0.5;
      marker.color.b = 0.5;

      for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it) {
        
        if (it->is_primary() && it->is_finite()){
          // we don't really care about the secondary ones since we can't use
          // them to navigate, and the infinite ones which are unfeasible
          if(it->is_linear()){
            auto edge = *it;
            geometry_msgs::msg::Point a = v2g_p(*edge.vertex0(), scale);
            geometry_msgs::msg::Point b = v2g_p(*edge.vertex1(), scale);
            marker.points.push_back(a);
            marker.points.push_back(b);

          }else{
            // is curved
            std::vector<BoostPoint> points;

            BoostPoint vertex0(it->vertex0()->x(), it->vertex0()->y());
            BoostPoint vertex1(it->vertex1()->x(), it->vertex1()->y());

            points.push_back(vertex0);
            points.push_back(vertex1);

            boost::polygon::voronoi_edge<double> edge = *it;

            // the process needs a segment and a specific point for the orientation
            BoostPoint point = edge.cell()->contains_point() ?
              retrieve_point(*edge.cell()) :
              retrieve_point(*edge.twin()->cell());
            BoostSegment segment = edge.cell()->contains_point() ?
              retrieve_segment(*edge.twin()->cell()) :
              retrieve_segment(*edge.cell());
            
            boost::polygon::voronoi_visual_utils<double>::discretize(
            point, segment, discretization, &points);

            for(int i = 0; i<points.size()-1; i++){
              geometry_msgs::msg::Point ma = b2g_p(points[i], scale);
              geometry_msgs::msg::Point mb = b2g_p(points[i+1], scale);
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

    void add_ids_to_vertices(){
      int id = 0;
      for(auto itr = vd.vertices().begin(); itr != vd.vertices().end(); ++itr){
        auto c = itr->color();
        c += id;
        itr->color(c);
        id++;
      }
    }

    void build_search_graph(){
      Graph graph;
      for(auto itr = vd.vertices().begin(); itr != vd.vertices().end(); ++itr){
        graph.add_node(itr->x(), itr->y());
      }
      for(auto itr = vd.edges().begin(); itr != vd.edges().end(); ++itr){
        if(itr->is_infinite() || itr->is_secondary())
          // I am not interested in infinite or secondary edges, we don't use
          // them for navigation 
          continue;
        
        auto v0 = itr->vertex0();
        auto v1 = itr->vertex1();
        if(itr->is_linear()){
          graph.add_edge(v0->color(), v1->color());   

        }else{
          // is curved
          // boost do not have lenght data of edges, so we need to discretize them
          std::vector<BoostPoint> points;

          BoostPoint vertex0(v0->x(), v0->y());
          BoostPoint vertex1(v1->x(), v1->y());

          points.push_back(vertex0);
          points.push_back(vertex1);

          boost::polygon::voronoi_edge<double> edge = *itr;

          // the process needs a specific half-segment and a specific point for the correct orientation
          BoostPoint point = edge.cell()->contains_point() ?
            retrieve_point(*edge.cell()) :
            retrieve_point(*edge.twin()->cell());
          BoostSegment segment = edge.cell()->contains_point() ?
            retrieve_segment(*edge.twin()->cell()) :
            retrieve_segment(*edge.cell());
          
          boost::polygon::voronoi_visual_utils<double>::discretize(
          point, segment, discretization, &points);

          if(points.size()==2){
            // the edge is curved, but small enough we do not need to divide it
            graph.add_edge(v0->color(), v1->color());
          }else{
            // the edge has been divided into multiple edges
            int new_nodes = points.size()-2;
            int first_new_index = graph.nodes.size();
            for(int i = 1; i<points.size()-1; i++){
              graph.add_node(points[i].x(), points[i].y());
            }
            int prev = v0->color();
            for(int i = 0; i<new_nodes; i++){
              graph.add_edge(prev,first_new_index+i);
              prev = first_new_index+i;
            }
            graph.add_edge(prev,v1->color());
          }
        }

      }
      
    }
    
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoadmapManager>());
  rclcpp::shutdown();
  return 0;
}
