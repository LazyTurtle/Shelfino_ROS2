#include <cstdio>
#include <memory>
#include <vector>
#include <set>
#include <queue>
#include <map>
#include <iomanip>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include "dubins_planner_msgs/msg/dubins_point.hpp"
#include "dubins_planner_msgs/srv/dubins_planning.hpp"
#include "dubins_planner_msgs/srv/multi_point_dubins_planning.hpp"

#include "roadmap_interfaces/srv/path_service.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
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
    std::map<int, double> edge_width;
    Node(){}
    Node(double in_x, double in_y) : x(in_x), y(in_y){}


};

class Graph{

  public:

    std::vector<Node> nodes;

    Graph(){}

    void add_node(double x, double y){
      Node n(x,y);
      nodes.push_back(n);
    }

    void clean_edges(){
      for(auto& node : nodes){
        node.neighbours.clear();
        node.edge_width.clear();
      }
    }

    void add_edges(const std::vector<std::pair<int, int>>& edges){
      for(auto& edge:edges)
        add_edge(edge);
    }

    void add_edge(std::pair<int, int> edge){
      add_edge(edge.first, edge.second);
    }

    void add_edge(std::pair<int, int> edge, double edge_width){
      add_edge(edge.first, edge.second, edge_width);
    }

    void add_edge(int node_a, int node_b){
      nodes[node_a].neighbours.insert(node_b);
      nodes[node_b].neighbours.insert(node_a);
    }

    void add_edge(int node_a, int node_b, double edge_width){
      nodes[node_a].neighbours.insert(node_b);
      nodes[node_b].neighbours.insert(node_a);
      nodes[node_a].edge_width[node_b] = edge_width;
      nodes[node_b].edge_width[node_a] = edge_width;
    }
  
    int find_closest(double x, double y, double minimum_width = 0.0){

      int closest = -1;
      double min_dist = std::numeric_limits<double>().infinity();
      for(std::size_t i=0; i<nodes.size(); i++){
        if(!has_edges_wide_enough(i, minimum_width))
          continue;
        double dist = Node::distance(x,y,nodes[i].x,nodes[i].y);
        if(dist<min_dist){
          min_dist = dist;
          closest = i;
        }
      }
      return closest;
    }

    std::vector<int> find_path(int start, int end, double tollerance = 0.0, double minimum_width = 0.0){

      auto find_minimum = [](const std::vector<int>& elements, const std::vector<double>& values){
        int min;
        double min_val = std::numeric_limits<double>().infinity();
        for(auto i:elements){
          double temp = values[i];
          if(temp<min_val){
            min_val = values[i];
            min = i;
          }
        }
        return min;
      };

      std::vector<double> heuristic, g_score, f_score;

      for(std::size_t i = 0; i<nodes.size(); i++){
        heuristic.push_back(Node::distance(nodes[end],nodes[i]));
      }

      g_score = std::vector<double>(nodes.size(), std::numeric_limits<double>().infinity());
      g_score[start] = 0;

      f_score = std::vector<double>(nodes.size(), std::numeric_limits<double>().infinity());
      f_score[start] = heuristic[start];

      std::map<int,int> came_from;
      std::vector<int> open_set;
      std::set<int> in_open_set; // I could use find, but this is clearer
      open_set.push_back(start);
      in_open_set.insert(start);

      while(!open_set.empty()){
        int current = find_minimum(open_set, f_score);

        if(current == end)
          return reconstruct_path(came_from, current, tollerance);

        open_set.erase(std::find(open_set.begin(), open_set.end(), current));
        in_open_set.erase(current);
        for(int neighbour:nodes[current].neighbours){
          // let's consider only neighbours which are reachable through a wide enough edge
          if(nodes[current].edge_width[neighbour]<minimum_width)
            continue;
          
          double temp_g_score = g_score[current] + Node::distance(nodes[current], nodes[neighbour]);

          if(temp_g_score<g_score[neighbour]){
            came_from[neighbour] = current;
            g_score[neighbour] = temp_g_score;
            f_score[neighbour] = g_score[neighbour] + heuristic[neighbour];
            if(in_open_set.count(neighbour)==0){
              open_set.push_back(neighbour);
              in_open_set.insert(neighbour);
            }
          }
        }
      }

      std::vector<int> empty_path;
      return empty_path;
    }

    std::vector<int> reconstruct_path(std::map<int,int>& came_from_map, int current, double tollerance = 0.0){
      std::vector<int> path;
      path.push_back(current);
      int last_valid_current = current;

      while(came_from_map.find(current)!=came_from_map.end()){
        current = came_from_map[current];

        if(Node::distance(nodes[current],nodes[last_valid_current]) > tollerance){
          path.insert(path.begin(), current);
          last_valid_current = current;
        }

      }
      return path;
    }

    bool has_edges_wide_enough(int node_index, double minimum_width){
      bool retult = false;
      for(auto map_item:nodes[node_index].edge_width){
        if(map_item.second>=minimum_width)
          retult = true;
      }
      return retult;
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

    const double MAXIMUM_CURVATURE = 4.0;
    const int DISCRETIZATION_DELTA = 4;
    const int REFINEMENTS = 3;
    const double MINIMUM_WAYPOINT_DISTANCE = 0.5;

    const std::string MAP_BORDER_TOPIC = "/map_borders";
    const std::string OBSTACLES_TOPIC = "/obstacles";
    const std::string GATES_TOPIC = "/gate_position";

    const std::string COMPUTE_PATH_SERVICE_NAME = "compute_path";

    const std::string MARKERS_TOPIC = "markers";
    const std::string WIDTHS_TOPIC = "widths";

    const std::string DUBINS_CALCULATOR_SERVICE = "/multi_points_dubins_calculator";

    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr border_subscriber;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gates_subscriber;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr widths_publisher;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Service<roadmap_interfaces::srv::PathService>::SharedPtr path_service;

    std::shared_ptr<geometry_msgs::msg::PolygonStamped> borders_msg;
    std::shared_ptr<obstacles_msgs::msg::ObstacleArrayMsg> obstacles_msg;
    std::shared_ptr<geometry_msgs::msg::PoseArray> gates_msg;

    visualization_msgs::msg::MarkerArray markers;
    enum markers_enum {voronoi, waypoints, obstacles, other_robots};
    int MARKERS_NUM = 4;

    visualization_msgs::msg::MarkerArray width_text;

    voronoi_diagram<double> vd;
    Graph search_graph;
    std::vector<BoostSegment> segments_data;
    std::vector<BoostPoint> points_data;

    int scale = 100;
    int discretization = 0.2 * scale;

    void node_setup(){
      
      const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

      border_subscriber = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
        MAP_BORDER_TOPIC, qos, std::bind(&RoadmapManager::set_borders, this, _1));
      
      obstacles_subscriber = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
        OBSTACLES_TOPIC, qos, std::bind(&RoadmapManager::set_obstacles, this, _1));

      gates_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        GATES_TOPIC, qos, std::bind(&RoadmapManager::set_gates, this, _1));

      path_service = this->create_service<roadmap_interfaces::srv::PathService>(
        COMPUTE_PATH_SERVICE_NAME, std::bind(&RoadmapManager::compute_path, this, _1, _2));

      markers_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        MARKERS_TOPIC, qos);

      widths_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        WIDTHS_TOPIC, qos);
      
      markers.markers = std::vector<visualization_msgs::msg::Marker>(MARKERS_NUM);

      auto interval = 1000ms;
      timer_ = this->create_wall_timer(interval, std::bind(&RoadmapManager::publish_data, this));
    }

    void log(std::string log_str){
      RCLCPP_INFO(this->get_logger(), log_str.c_str());
    }

    void err(std::string log_str){
      RCLCPP_ERROR(this->get_logger(), log_str.c_str());
    }

    void set_borders(const geometry_msgs::msg::PolygonStamped::SharedPtr msg){
      borders_msg = msg;
    }

    void set_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg){
      obstacles_msg = msg;
      add_obstacles_markers();
    }

    void set_gates(const geometry_msgs::msg::PoseArray::SharedPtr msg){
      gates_msg = msg;
    }

    void publish_data(){
      markers_publisher->publish(markers);
      widths_publisher->publish(width_text);
    }

    void compute_path(
      const std::shared_ptr<roadmap_interfaces::srv::PathService_Request> request,
      std::shared_ptr<roadmap_interfaces::srv::PathService_Response> response){
      
      log("Start calculating a new path.");
      if(!update_voronoi_diagram()){
        err("Cannot update the voronoi diagram");
      }
      
      double start_x = request->start.x;
      double start_y = request->start.y;
      double end_x = request->end.x;
      double end_y = request->end.y;
      double minimum_width = request->minimum_path_width;


      int closest_node_to_start = search_graph.find_closest(start_x, start_y, minimum_width);
      int closest_node_to_end = search_graph.find_closest(end_x, end_y, minimum_width);

      if(closest_node_to_start < 0 || closest_node_to_end< 0){
        err("Error while looking for the closest nodes.");
        return;
      }

      std::vector<int> path_int = search_graph.find_path(
        closest_node_to_start, closest_node_to_end, MINIMUM_WAYPOINT_DISTANCE, minimum_width);

      if(path_int.size()==0){
        std::ostringstream s;
        s<<"No path found from ["<<start_x<<","<<start_y<<"] to ["<<end_x<<","<<end_y<<"].";
        log(s.str());
        return;
      }

      std::shared_ptr<rclcpp::Node> client_node = rclcpp::Node::make_shared("dubins_client");
      rclcpp::Client<dubins_planner_msgs::srv::MultiPointDubinsPlanning>::SharedPtr client =
        client_node->create_client<dubins_planner_msgs::srv::MultiPointDubinsPlanning>(DUBINS_CALCULATOR_SERVICE);
      
      auto r = std::make_shared<dubins_planner_msgs::srv::MultiPointDubinsPlanning::Request>();

      std::vector<geometry_msgs::msg::Point> path_geo;
      geometry_msgs::msg::Point p;
      p.x = start_x;
      p.y = start_y;
      path_geo.push_back(p);
      for(int i:path_int){
        geometry_msgs::msg::Point p;
        p.x = search_graph.nodes[i].x;
        p.y = search_graph.nodes[i].y;
        path_geo.push_back(p);
      }
      p.x = end_x;
      p.y = end_y;
      path_geo.push_back(p);

      r->points = path_geo;
      r->kmax = MAXIMUM_CURVATURE;
      r->komega = DISCRETIZATION_DELTA;
      r->refinements = REFINEMENTS;

      while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        err("Interrupted while waiting for the service. Exiting.");
        return;
        }
        log("service not available, waiting again...");
      }

      auto result = client->async_send_request(r);
      
      if (rclcpp::spin_until_future_complete(client_node, result) == rclcpp::FutureReturnCode::SUCCESS){
        auto temp = result.get();        
        response->path = temp->path;
        response->path.header.frame_id = "map";
        response->path.header.stamp = this->now();
        response->result = true;
        log("Path calculated, length: "+std::to_string(temp->lenght));
        return;
      } else {
        err("Failed to call service: "+DUBINS_CALCULATOR_SERVICE);
        return;
      }
    }

    bool update_voronoi_diagram(){

      if(!borders_msg || !obstacles_msg){
        log("Pointers not ready.");
        return false;
      }
      log("Start updating the voronoi diagram.");

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
      update_diagram_markers();
      add_width_markers(search_graph);

      return true;
    }

    void add_polygon_to_boost_segments(
      const geometry_msgs::msg::Polygon& poly, std::vector<BoostSegment>& segments){

      for(std::size_t i = 0; i<poly.points.size()-1; i++){
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

    void update_diagram_markers(){
      std::vector<geometry_msgs::msg::Point> nodes_coordinates;
      visualization_msgs::msg::Marker marker;

      marker.header.stamp = this->now();
      marker.header.frame_id = "map";
      marker.id = markers_enum::voronoi;
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
          // this actually create twice as many segments as needed
          // but it works fine and it's a pain to use BFS only for this
          geometry_msgs::msg::Point mb;
          mb.x = search_graph.nodes[neighbour].x;
          mb.y = search_graph.nodes[neighbour].y;
          marker.points.push_back(ma);
          marker.points.push_back(mb);
        }

      }

      markers.markers[markers_enum::voronoi] = marker;
      log("Updated voronoi graph marker.");
      add_points_markers(nodes_coordinates, markers_enum::waypoints);
    }

    void add_points_markers(
      const std::vector<geometry_msgs::msg::Point>& point_list,
      markers_enum type = markers_enum::waypoints, double l = 0.1,
      double r=0.5, double g=0.5, double b=0.5){
      visualization_msgs::msg::Marker marker;

      marker.header.stamp = this->now();
      marker.header.frame_id = "map";
      marker.id = type;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.scale.x = l;
      marker.scale.y = l;
      marker.scale.z = 0.1;
      marker.color.a = 1.0;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;

      marker.points = point_list;

      markers.markers[type] = marker;
      log("Updated waypoints marker at "+std::to_string(type)+".");
    }

    void add_obstacles_markers(){
      std::vector<geometry_msgs::msg::Polygon> polygons;
      for(auto obs:obstacles_msg->obstacles){
        polygons.push_back(obs.polygon);
      }
      add_obstacles_markers(polygons, markers_enum::obstacles);
    }

    void add_obstacles_markers(
      const std::vector<geometry_msgs::msg::Polygon> obstacles, markers_enum type){
      
      auto tp = [](const geometry_msgs::msg::Point32 p){
        geometry_msgs::msg::Point a;
        a.x = p.x;
        a.y = p.y;
        a.z = p.z;
        return a;
      };
      auto extract_segment = [tp](const geometry_msgs::msg::Polygon polygon){
        std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> segments;
        for(std::size_t i = 0; i<polygon.points.size(); i++){
          geometry_msgs::msg::Point a = tp(polygon.points[i]);
          geometry_msgs::msg::Point b = tp(polygon.points[(i+1)%polygon.points.size()]);
          segments.push_back(std::make_pair(a,b));
        }
        return segments;
      };

      visualization_msgs::msg::Marker marker;
      marker.header.stamp = this->now();
      marker.header.frame_id = "map";
      marker.id = type;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker.scale.x = 0.05;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 0.9;
      marker.color.g = 0.1;
      marker.color.b = 0.1;

      for(auto obs:obstacles){
        auto segments = extract_segment(obs);
        for(auto seg:segments){
          marker.points.push_back(seg.first);
          marker.points.push_back(seg.second);
        }
      }

      markers.markers[type] = marker;
      log("Updated obstacles markers at "+std::to_string(type));
    }

    void add_width_markers(Graph& graph){
      visualization_msgs::msg::MarkerArray text_array;
      int id = 0;
      for(std::size_t i=0; i<graph.nodes.size(); i++){
        for(int j:graph.nodes[i].neighbours){
          visualization_msgs::msg::Marker marker;
          marker.header.stamp = this->now();
          marker.header.frame_id = "map";
          marker.id = id;
          id++;
          marker.action = visualization_msgs::msg::Marker::ADD;
          marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          marker.scale.x = 0.2;
          marker.scale.y = 0.2;
          marker.scale.z = 0.2;
          marker.color.a = 1.0;
          marker.color.r = 1;
          marker.color.g = 1;
          marker.color.b = 1;
          
          double d = graph.nodes[i].edge_width[j];
          std::ostringstream ss;
          ss << std::fixed << std::setprecision(3) << d;
          marker.text = ss.str();

          double x = (graph.nodes[i].x + graph.nodes[j].x)/2.0;
          double y = (graph.nodes[i].y + graph.nodes[j].y)/2.0;
          marker.pose.position.x = x;
          marker.pose.position.y = y;
          marker.pose.position.z = 0.1;
          
          
          text_array.markers.push_back(marker);
        }
      }
      width_text = text_array;
      log("Updated width markers");
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
        double v_x = itr->x()/scale;
        double v_y = itr->y()/scale;
        graph.add_node(v_x, v_y);
      }
      for(auto itr = vd.edges().begin(); itr != vd.edges().end(); ++itr){
        if(itr->is_infinite() || itr->is_secondary())
          // I am not interested in infinite or secondary edges, we don't use
          // them for navigation 
          continue;

        auto v0 = itr->vertex0();
        auto v1 = itr->vertex1();
        if(itr->is_linear()){
          
          double edge_width = find_edge_width(*itr, scale);
          graph.add_edge(v0->color(), v1->color(), edge_width);

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

          // TODO: I have no idea how to handle very long curves
          // I only have data about the end points and all I can do are averages
          // or being conservative and always choose the smallest one
          double edge_width = find_edge_width(*itr, scale);

          if(points.size()==2){
            // the edge is curved, but small enough we do not need to divide it
            graph.add_edge(v0->color(), v1->color(), edge_width);
          }else{
            // the edge has been divided into multiple edges
            int new_nodes = points.size()-2;
            int first_new_index = graph.nodes.size();
            for(std::size_t i = 1; i<points.size()-1; i++){
              graph.add_node(points[i].x()/scale, points[i].y()/scale);
            }
            int prev = v0->color();
            for(int i = 0; i<new_nodes; i++){
              graph.add_edge(prev,first_new_index+i, edge_width);
              prev = first_new_index+i;
            }
            graph.add_edge(prev,v1->color(), edge_width);
          }
        }
      }

      return graph;
    }

    double find_edge_width(boost::polygon::voronoi_edge<double> edge, double scale){
      // small functions to help here
      auto dist = [](double ax, double ay, double bx, double by){
        double dist = std::sqrt(std::pow(ax - bx, 2)+std::pow(ay - by,2));
        return dist;
      };
      auto dist_pe = [dist, scale](BoostPoint& point, boost::polygon::voronoi_edge<double>& edge){
        double mean_x = (edge.vertex0()->x() + edge.vertex1()->x())/2.0;
        double mean_y = (edge.vertex0()->y() + edge.vertex1()->y())/2.0;
        double distance = dist(point.x(), point.y(), mean_x, mean_y)/scale;
        return distance;
      };
      auto dist_ee = [dist, scale](boost::polygon::voronoi_edge<double>& a, boost::polygon::voronoi_edge<double>& b){
        double a_x = (a.vertex0()->x() + a.vertex1()->x())/2.0/scale;
        double a_y = (a.vertex0()->y() + a.vertex1()->y())/2.0/scale;
        double b_x = (b.vertex0()->x() + b.vertex1()->x())/2.0/scale;
        double b_y = (b.vertex0()->y() + b.vertex1()->y())/2.0/scale;
        double distance = dist(a_x, a_y, b_x, b_y);
        return distance;
      };
      auto min_dist_pe = [dist, scale](BoostPoint& point, boost::polygon::voronoi_edge<double>& edge){
        double dist_0 = dist(point.x(), point.y(), edge.vertex0()->x(), edge.vertex0()->y())/scale;
        double dist_1 = dist(point.x(), point.y(), edge.vertex1()->x(), edge.vertex1()->y())/scale;
        double minimum = std::min(dist_0, dist_1);
        return minimum;
      };
      auto min_dist_ee = [dist, scale](boost::polygon::voronoi_edge<double>& a, boost::polygon::voronoi_edge<double>& b){
        double dist_00 = dist(a.vertex0()->x(), a.vertex0()->y(), b.vertex0()->x(), b.vertex0()->y())/scale;
        double dist_01 = dist(a.vertex0()->x(), a.vertex0()->y(), b.vertex1()->x(), b.vertex1()->y())/scale;
        double dist_10 = dist(a.vertex1()->x(), a.vertex1()->y(), b.vertex0()->x(), b.vertex0()->y())/scale;
        double dist_11 = dist(a.vertex1()->x(), a.vertex1()->y(), b.vertex1()->x(), b.vertex1()->y())/scale;
        auto min_v = {dist_00, dist_01, dist_10, dist_11};
        auto minimum = std::min_element(min_v.begin(), min_v.end());
        return minimum;
      };

      auto min_distance_from_cells = [this, dist, dist_pe, dist_ee, min_dist_pe, min_dist_ee]
      (boost::polygon::voronoi_cell<double>* cell, boost::polygon::voronoi_edge<double>& edge){
        if (cell->contains_point()) {
          if(cell->source_category()==boost::polygon::SOURCE_CATEGORY_SINGLE_POINT) {
            std::size_t index = cell->source_index();
            BoostPoint p = points_data[index];
            double d = min_dist_pe(p,edge);
            return d;

          }else if(cell->source_category()==boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT) {
            std::size_t index = cell->source_index() - points_data.size();
            BoostPoint p0 = low(segments_data[index]);
            double d = min_dist_pe(p0, edge);
            return d;

          }else if(cell->source_category()==boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT) {
            std::size_t index = cell->source_index() - points_data.size();
            BoostPoint p1 = high(segments_data[index]);
            double d = min_dist_pe(p1, edge);
            return d;
          }
        }else{
          std::size_t index = cell->source_index() - points_data.size();
          BoostPoint p0 = low(segments_data[index]);
          BoostPoint p1 = high(segments_data[index]);
          double dist0 = min_dist_pe(p0, edge);
          double dist1 = min_dist_pe(p1, edge);
          double min = std::min(dist0, dist1);
          return min;
        }
        return -1.0;
      };
      
      boost::polygon::voronoi_cell<double>* cell_a = edge.cell();
      boost::polygon::voronoi_cell<double>* cell_b = edge.twin()->cell();

      double dist_a = min_distance_from_cells(cell_a, edge);
      double dist_b = min_distance_from_cells(cell_b, edge);
      double min_dist = std::min(dist_a, dist_b);
      return min_dist;
      
    }
    
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoadmapManager>());
  rclcpp::shutdown();
  return 0;
}
