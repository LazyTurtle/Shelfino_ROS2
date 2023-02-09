#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/wait_for_message.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "roadmap_interfaces/srv/path_service.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

class RobotDriver : public rclcpp::Node
{
  public:
    std::shared_ptr<nav_msgs::msg::Path> path;
    double path_length;


    RobotDriver()
    : Node("robot_driver"){}

    void init(){
      const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

      gates_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        GATES_TOPIC, qos, std::bind(&RobotDriver::set_gates, this, _1));
    
      robot_position_subscriber = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        ROBOT_POSITION_TOPIC, qos, std::bind(&RobotDriver::set_robot_position, this, _1));

      test_service = this->create_service<std_srvs::srv::Empty>(
        "test_drive", std::bind(&RobotDriver::find_best_path, this, _1, _2));


      path_publisher = this->create_publisher<nav_msgs::msg::Path>(PATH_TOPIC, qos);

      timer = this->create_wall_timer(publishers_period, std::bind(&RobotDriver::publish, this));
      
      robot_id = get_robot_id();
      log("Driver "+std::to_string(robot_id)+" ready.");
    }

  private:

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr test_service;

    const double ROBOT_WIDTH = 0.5;
    const int N_ROBOTS = 3;

    const std::chrono::milliseconds publishers_period = 1000ms;
    rclcpp::TimerBase::SharedPtr timer;

    const std::string ROADMAP_SERVICE_NAME = "compute_path";
    const std::string GATES_TOPIC = "/gate_position";
    const std::string ROBOT_POSITION_TOPIC = "transform";
    const std::string PATH_TOPIC = "shortest_path";

    int robot_id;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gates_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr robot_position_subscriber;

    std::shared_ptr<geometry_msgs::msg::PoseArray> gates;
    std::shared_ptr<geometry_msgs::msg::TransformStamped> robot_position;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;


    void log(std::string log){
      RCLCPP_INFO(this->get_logger(), log.c_str());
    }

    void err(std::string log){
      RCLCPP_ERROR(this->get_logger(), log.c_str());
    }
    
    void set_gates(const geometry_msgs::msg::PoseArray::SharedPtr msg){
      gates = msg;
    }

    void set_robot_position(const geometry_msgs::msg::TransformStamped::SharedPtr msg){
      robot_position = msg;
    }

    int get_robot_id(){
      std::string ns = this->get_namespace();
      std::string s(1,ns.back());
      int id = std::stoi(s);
      return id;
    }

    void publish(){
      if(path)
        path_publisher->publish(*path);
    }

    void find_best_path(
      const std::shared_ptr<std_srvs::srv::Empty_Request> request,
      std::shared_ptr<std_srvs::srv::Empty_Response> response){
      get_path();
      follow_path();
    }

    void follow_path(){

      rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr client_ptr;
      client_ptr = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(this,"follow_path");
      if (!client_ptr->wait_for_action_server()) {
      err("Action server not available after waiting");
      rclcpp::shutdown();
      }
      auto goal_msg = nav2_msgs::action::FollowPath::Goal();
      goal_msg.path = *path;
      goal_msg.controller_id = "FollowPath";
      RCLCPP_INFO(this->get_logger(), "Sending goal");
      client_ptr->async_send_goal(goal_msg);
    }

    void get_path(){
      
      if(!gates){
        err("Gates pointer is null");
        return;
      }
      if(!robot_position){
        err("The robot position is null");
        return;
      }

      auto length_of_path = [](const nav_msgs::msg::Path& path){
        double length = 0.0;
        for(std::size_t i=1; i<path.poses.size(); i++){
          double dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
          double dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
          double dz = path.poses[i].pose.position.z - path.poses[i-1].pose.position.z;
          double dd = std::sqrt(std::pow(dx, 2.0)+std::pow(dy, 2.0)+std::pow(dz, 2.0));
          length += dd;
        }
        return length;
      };

      std::shared_ptr<rclcpp::Node> client_node = rclcpp::Node::make_shared(ROADMAP_SERVICE_NAME+"_client");
      auto client = client_node->create_client<roadmap_interfaces::srv::PathService>(ROADMAP_SERVICE_NAME);
      
      std::vector<nav_msgs::msg::Path> possible_paths;
      auto obstacles = obstacles_from_robots();

      for(auto gate:gates->poses){
        {
          std::ostringstream s;
          s<<"Lookign for path to x:"<<gate.position.x<<" y:"<<gate.position.y;
          log(s.str());
        }

        auto req = std::make_shared<roadmap_interfaces::srv::PathService::Request>();

        req->start.x = robot_position->transform.translation.x;
        req->start.y = robot_position->transform.translation.y;
        req->start.z = robot_position->transform.translation.z;

        req->end.x = gate.position.x;
        req->end.y = gate.position.y;
        req->end.z = gate.position.z;

        // actually, half width plus a small tollerance
        req->minimum_path_width = (ROBOT_WIDTH / 2.0) + 0.1;

        req->obstacles = obstacles;

        while (!client->wait_for_service(1s)){
          if (!rclcpp::ok()){
            err("Interrupted while waiting for the service. Exiting.");
            return;
          }
          log("service not available, waiting again...");
        }
        log("Requesting path from roadmap manager.");
        auto result = client->async_send_request(req);

        if (rclcpp::spin_until_future_complete(client_node, result) == rclcpp::FutureReturnCode::SUCCESS){
          auto res = result.get();

          if(res->result)
            possible_paths.push_back(res->path);
          else
            err("There was no availabe path.");
        }else{
          err("Failed to call service "+ROADMAP_SERVICE_NAME);
        }
      }

      std::vector<double> lengths;
      for(auto path:possible_paths){
        lengths.push_back(length_of_path(path));
      }

      auto min_itr = std::min_element(lengths.begin(), lengths.end());
      int min_index = std::distance(lengths.begin(), min_itr);

      path = std::make_shared<nav_msgs::msg::Path>(possible_paths[min_index]);
      path_length = lengths[min_index];
      log("Shortest path obtained.");

    }

    std::vector<geometry_msgs::msg::Polygon> obstacles_from_robots(){
      std::vector<geometry_msgs::msg::Polygon> obstacles;
      std::vector<geometry_msgs::msg::TransformStamped> transforms = get_robot_transforms();
      for(auto robot_tr:transforms){
        geometry_msgs::msg::Polygon obs = create_square(robot_tr, ROBOT_WIDTH);
        obstacles.push_back(obs);
      }
      return obstacles;
    }
    
    std::vector<geometry_msgs::msg::TransformStamped> get_robot_transforms(){
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
      
      std::vector<geometry_msgs::msg::TransformStamped> transforms;
      for(int i = 0; i<N_ROBOTS+1; i++){
        if(i == robot_id)
          continue;
        geometry_msgs::msg::TransformStamped t;
        std::string topic_name = "/shelfino"+std::to_string(i)+"/transform";
        log("Looking for topic:"+topic_name);

        // this is necessary if we want to use qos
        std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::TransformStamped>>
         sub = this->create_subscription<geometry_msgs::msg::TransformStamped>
          (topic_name,qos,[](const std::shared_ptr<const geometry_msgs::msg::TransformStamped>){});

        bool obtained = rclcpp::wait_for_message(t,sub, this->get_node_options().context(), 1s);
        if(obtained){
          transforms.push_back(t);
          log("found transform for shelfino"+std::to_string(i));
        }
      }
      log("Found "+std::to_string(transforms.size())+" transforms.");
      return transforms;
    }

    geometry_msgs::msg::Polygon create_square(
      const geometry_msgs::msg::TransformStamped transform, const double robot_width){
        geometry_msgs::msg::Polygon p;
        double h = (robot_width/2.0);
        const double x = transform.transform.translation.x;
        const double y = transform.transform.translation.y;
        tf2::Quaternion q;
        tf2::fromMsg(transform.transform.rotation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getEulerYPR(yaw, pitch, roll);
        std::vector<geometry_msgs::msg::Point32> points(4);
        points[0].x = h*std::cos(yaw) - h*std::sin(yaw) +x;
        points[0].y = h*std::sin(yaw) + h*std::cos(yaw) +y;
        points[1].x = -h*std::cos(yaw) - h*std::sin(yaw) +x;
        points[1].y = -h*std::sin(yaw) + h*std::cos(yaw) +y;
        points[2].x = -h*std::cos(yaw) - (-h)*std::sin(yaw) +x;
        points[2].y = -h*std::sin(yaw) + (-h)*std::cos(yaw) +y;
        points[3].x = h*std::cos(yaw) - (-h)*std::sin(yaw) +x;
        points[3].y = h*std::sin(yaw) + (-h)*std::cos(yaw) +y;
        
        p.points = points;

        for(auto po:points){
          std::ostringstream s;
          s<<"x:"<<po.x<<" y:"<<po.y;
          log(s.str());
        }
        
        return p;
      }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto pub = std::make_shared<RobotDriver>();
  pub->init();
  rclcpp::spin(pub);
  rclcpp::shutdown();
  
  return 0;
}