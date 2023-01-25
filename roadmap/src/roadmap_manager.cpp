#include <cstdio>
#include <memory>
#include <vector>
#include <set>
#include <queue>
#include <map>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include "dubins_planner_msgs/msg/dubins_point.hpp"
#include "dubins_planner_msgs/srv/dubins_planning.hpp"
#include "dubins_planner_msgs/srv/multi_point_dubins_planning.hpp"

#include "roadmap_interfaces/srv/path_service.hpp"
#include "visualization_msgs/msg/marker.hpp"  
#include "nav_msgs/msg/path.hpp"
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
  public:
    static double distance(Node& a, Node& b){
      return distance(a.x, a.y, b.x, b.y); 
    }
    static double distance(double ax, double ay, double bx, double by){
      double x = ax - bx;
      double y = ay - by;
      double d = std::sqrt(std::pow(x,2)+std::pow(y,2)); 
      return d; 
    }

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
  
    int find_closest(double x, double y){
      int closest = -1;
      double min_dist = std::numeric_limits<double>().infinity();
      for(int i=0; i<nodes.size(); i++){
        double dist = Node::distance(x,y,nodes[i].x,nodes[i].y);
        if(dist<min_dist){
          min_dist = dist;
          closest = i;
        }
      }
      return closest;
    }

    std::vector<int> find_path(int start, int end){
      class Estimate{
        bool reverse;
        public:
          // optimistic estimate of the distance between node n and node end
          std::vector<double> heuristic;
          // current minimum distance from node start to node n
          std::vector<double> g_score;
          // current best guess from start to finish, = g_score + heuristic
          std::vector<double> f_score;

          Estimate(int start, int end, Graph* graph, const bool& revparam=false){
            reverse=revparam;

            for(int i = 0; i<graph->nodes.size(); i++){
              heuristic.push_back(Node::distance(graph->nodes[end],graph->nodes[i]));
            }

            g_score = std::vector<double>(graph->nodes.size(), std::numeric_limits<double>().infinity());
            g_score[start] = 0;

            f_score = std::vector<double>(graph->nodes.size(), std::numeric_limits<double>().infinity());
            f_score[start] = heuristic[start];
          }

          bool operator() (const int& lhs, const int& rhs) const
          {
            if (reverse) return (f_score[lhs]>f_score[rhs]);
            else return (f_score[lhs]<f_score[rhs]);
          }
      };

      std::map<int,int> came_from;
      Estimate e(start, end, this);
      std::priority_queue<int, std::vector<int>, Estimate> open_set(e);
      std::set<int> in_open_set; // I need this only because priority_queue does not have lookup methods
      open_set.push(start);
      in_open_set.insert(start);

      while(!open_set.empty()){
        int current = open_set.top();

        if(current == end)
          return reconstruct_path(came_from, current);

        open_set.pop();
        in_open_set.erase(current);
        for(int neighbour:nodes[current].neighbours){
          double temp_g_score = e.g_score[current] + Node::distance(nodes[current], nodes[neighbour]);

          if(temp_g_score<e.g_score[neighbour]){
            came_from[neighbour] = current;
            e.g_score[neighbour] = temp_g_score;
            e.f_score[neighbour] = e.g_score[neighbour] + e.heuristic[neighbour];
            if(in_open_set.count(neighbour)==0){
              open_set.push(neighbour);
              in_open_set.insert(neighbour);
            }
          }
        }
      }

      std::vector<int> empty_path;
      return empty_path;
    }
    std::vector<int> reconstruct_path(std::map<int,int>& came_from_map, int current){
      std::vector<int> path;
      path.push_back(current);
      while(came_from_map.find(current)!=came_from_map.end()){
        current = came_from_map[current];
        path.insert(path.begin(), current);
      }
      return path;
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
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoints_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Service<roadmap_interfaces::srv::PathService>::SharedPtr path_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr test_service;

    std::shared_ptr<geometry_msgs::msg::PolygonStamped> borders_msg;
    std::shared_ptr<obstacles_msgs::msg::ObstacleArrayMsg> obstacles_msg;
    std::shared_ptr<geometry_msgs::msg::PoseArray> gates_msg;

    visualization_msgs::msg::Marker vd_marker;
    visualization_msgs::msg::Marker waypoints_marker;
    nav_msgs::msg::Path calculated_path; 

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
      
      waypoints_publisher = this->create_publisher<visualization_msgs::msg::Marker>(
        "waypoints", qos);

      path_publisher = this->create_publisher<nav_msgs::msg::Path>(
        "path", qos);

      auto interval = 1000ms;
      timer_ = this->create_wall_timer(interval, std::bind(&RoadmapManager::publish_data, this));
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

    void publish_data(){
      diagram_publisher->publish(vd_marker);
      waypoints_publisher->publish(waypoints_marker);
      path_publisher->publish(calculated_path);
    }

    void test(
      const std::shared_ptr<std_srvs::srv::Empty_Request> request,
      std::shared_ptr<std_srvs::srv::Empty_Response> response){
      update_voronoi_diagram();

      int closest_node_to_start = search_graph.find_closest(-1, -4);
      int closest_node_to_end = search_graph.find_closest(4, 2);
      std::vector<int> path_int = search_graph.find_path(closest_node_to_start, closest_node_to_end);

      std::shared_ptr<rclcpp::Node> client_node = rclcpp::Node::make_shared("multi_points_dubins_calculator_client");
      rclcpp::Client<dubins_planner_msgs::srv::MultiPointDubinsPlanning>::SharedPtr client =
        client_node->create_client<dubins_planner_msgs::srv::MultiPointDubinsPlanning>("multi_points_dubins_calculator");
      
      auto r = std::make_shared<dubins_planner_msgs::srv::MultiPointDubinsPlanning::Request>();

      std::vector<geometry_msgs::msg::Point> path_geo;
      geometry_msgs::msg::Point p;
      p.x = -1;
      p.y = -4;
      path_geo.push_back(p);
      for(int i:path_int){
        geometry_msgs::msg::Point p;
        p.x = search_graph.nodes[i].x;
        p.y = search_graph.nodes[i].y;
        path_geo.push_back(p);
      }
      p.x = 4;
      p.y = 2;
      path_geo.push_back(p);

      //update_waypoints_marker(path_geo);

      r->points = path_geo;
      r->kmax = 6;
      r->komega = 8;

      while (!client->wait_for_service(3s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      auto result = client->async_send_request(r);
      // Wait for the result.
      if (rclcpp::spin_until_future_complete(client_node, result) == rclcpp::FutureReturnCode::SUCCESS){
        calculated_path = result.get()->path;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
      }

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

      add_ids_to_vertices();
      search_graph = build_search_graph();
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
      double x_start = request->start.x;
      double y_start = request->start.y;
      double x_end = request->end.x;
      double y_end = request->end.y;

      int closest_node_to_start = search_graph.find_closest(-1, -4);
      int closest_node_to_end = search_graph.find_closest(4, 2);
      std::vector<int> path = search_graph.find_path(closest_node_to_start, closest_node_to_end);

      std::vector<std::pair<double,double>> path_coordinates;
      path_coordinates.push_back(std::make_pair(x_start, y_start));
      for(int i : path){
        double x = search_graph.nodes[i].x;
        double y = search_graph.nodes[i].y;
        path_coordinates.push_back(std::make_pair(x, y));
      }
      path_coordinates.push_back(std::make_pair(x_end, y_end));

      calculated_path = refine_path(path_coordinates);


    }

    nav_msgs::msg::Path refine_path(const std::vector<std::pair<double,double>>& coordinates){
      nav_msgs::msg::Path refined_path;
      
      return refined_path;
    }

    void update_diagram_marker(){
      std::vector<geometry_msgs::msg::Point> nodes_coordinates;
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

      for(auto node:search_graph.nodes){
        geometry_msgs::msg::Point ma;
        ma.x = node.x;
        ma.y = node.y;
        nodes_coordinates.push_back(ma);
        for(auto neighbour:node.neighbours){
          // TODO: use BFS in order to only produce a single segment for edge
          geometry_msgs::msg::Point mb;
          mb.x = search_graph.nodes[neighbour].x;
          mb.y = search_graph.nodes[neighbour].y;
          marker.points.push_back(ma);
          marker.points.push_back(mb);
        }

      }

      vd_marker = marker;
      log("Updated voronoi graph marker.");
      update_waypoints_marker(nodes_coordinates);
    }

    void update_waypoints_marker(const std::vector<geometry_msgs::msg::Point>& point_list){
      visualization_msgs::msg::Marker marker;

      marker.header.stamp = this->now();
      marker.header.frame_id = "map";
      marker.id = 1;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.5;
      marker.color.b = 0.5;

      marker.points = point_list;

      waypoints_marker = marker;
      log("Updated waypoints marker.");
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

    Graph build_search_graph(){
      Graph graph;
      for(auto itr = vd.vertices().begin(); itr != vd.vertices().end(); ++itr){
        graph.add_node(itr->x()/scale, itr->y()/scale);
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
              graph.add_node(points[i].x()/scale, points[i].y()/scale);
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
      return graph;
    }
    
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoadmapManager>());
  rclcpp::shutdown();
  return 0;
}
