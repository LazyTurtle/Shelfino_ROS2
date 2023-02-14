#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

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
#include "roadmap_interfaces/srv/driver_service.hpp"
#include "roadmap_interfaces/action/evacuate.hpp"


using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

class RobotDriver : public rclcpp::Node
{
  public:
    std::shared_ptr<nav_msgs::msg::Path> path;
    double path_length;
    int gate_id;
    double start_delay = -1.0;


    RobotDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("robot_driver", options){}

    void init(){
      const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

      gates_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        GATES_TOPIC, qos, std::bind(&RobotDriver::set_gates, this, _1));
    
      best_path_service = this->create_service<roadmap_interfaces::srv::DriverService>(
        FIND_BEST_PATH_SERVICE, std::bind(&RobotDriver::find_best_path, this, _1, _2));

      evacuate_service = this->create_service<std_srvs::srv::Empty>(
        EVACUATE_SERVICE, std::bind(&RobotDriver::evacuate, this, _1, _2));

      path_publisher = this->create_publisher<nav_msgs::msg::Path>(PATH_TOPIC, qos);

      timer = this->create_wall_timer(publishers_period, std::bind(&RobotDriver::publish, this));

      this->action_server_ = rclcpp_action::create_server<roadmap_interfaces::action::Evacuate>(
      this, "action_evacuate",
      std::bind(&RobotDriver::handle_evacuation_goal, this, _1, _2),
      std::bind(&RobotDriver::handle_cancel, this, _1),
      std::bind(&RobotDriver::handle_accepted, this, _1));
      
      robot_id = get_robot_id();
      log("Driver "+std::to_string(robot_id)+" ready.");
    }

  private:

    const double ROBOT_WIDTH = 0.6;
    const int N_ROBOTS = 3;

    const std::chrono::milliseconds publishers_period = 1000ms;
    rclcpp::TimerBase::SharedPtr timer;

    const std::string ROADMAP_SERVICE_NAME = "compute_path";
    const std::string GATES_TOPIC = "/gate_position";
    const std::string ROBOT_POSITION_TOPIC = "transform";
    const std::string PATH_TOPIC = "shortest_path";

    const std::string FIND_BEST_PATH_SERVICE = "find_best_path";
    const std::string EVACUATE_SERVICE = "evacuate";

    int robot_id;

    bool b_action_ended = false;

    rclcpp::Service<roadmap_interfaces::srv::DriverService>::SharedPtr best_path_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr evacuate_service;

    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr follow_path_client;

    rclcpp_action::Server<roadmap_interfaces::action::Evacuate>::SharedPtr action_server_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gates_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr robot_position_subscriber;

    std::shared_ptr<geometry_msgs::msg::PoseArray> gates;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;

    void debug(std::string log){
      RCLCPP_DEBUG(this->get_logger(), log.c_str());
    }

    void log(std::string log){
      RCLCPP_INFO(this->get_logger(), log.c_str());
    }

    void err(std::string log){
      RCLCPP_ERROR(this->get_logger(), log.c_str());
    }
    
    void set_gates(const geometry_msgs::msg::PoseArray::SharedPtr msg){
      gates = msg;
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
      const std::shared_ptr<roadmap_interfaces::srv::DriverService_Request> request,
      std::shared_ptr<roadmap_interfaces::srv::DriverService_Response> response){
      log("Requested the best path.");
      if(self_shelfino_exist()){

        path_length = get_path(request->look_for_robots);
        response->length = path_length;
        response->result = (path_length>=0.0) ? true : false;
        response->gate_id = gate_id;
      }else{
        err("Shelfino does not exist");
        response->result = false;
      }
    }

    bool self_shelfino_exist(){
      auto t = obtain_transform_of_shelfino(robot_id);
      bool shelfino_exist = (!t.header.frame_id.empty());
      return shelfino_exist;
    }

    void evacuate(
      const std::shared_ptr<std_srvs::srv::Empty_Request> request,
      std::shared_ptr<std_srvs::srv::Empty_Response> response){
        follow_path();
      }

    double get_path(bool look_for_robots = true){
      
      if(!gates){
        err("Gates pointer is null");
        return -1.0;
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
      std::vector<geometry_msgs::msg::Polygon> obstacles = (look_for_robots)?
        obstacles_from_robots():
        std::vector<geometry_msgs::msg::Polygon>();

      auto robot_position = obtain_transform_of_shelfino(robot_id);

      // the robot does not exist anymore
      if(robot_position.header.frame_id.empty())
        return -1.0;

      for(auto const& gate:gates->poses){
        {
          std::ostringstream s;
          s<<"Lookign for path to x:"<<gate.position.x<<" y:"<<gate.position.y;
          log(s.str());
        }

        auto req = std::make_shared<roadmap_interfaces::srv::PathService::Request>();

        req->start.x = robot_position.transform.translation.x;
        req->start.y = robot_position.transform.translation.y;
        req->start.z = robot_position.transform.translation.z;

        req->end.x = gate.position.x;
        req->end.y = gate.position.y;
        req->end.z = gate.position.z;

        // actually, half width plus a small tollerance
        req->minimum_path_width = (ROBOT_WIDTH / 2.0) + 0.10;

        req->obstacles = obstacles;

        tf2::Quaternion q;
        tf2::fromMsg(robot_position.transform.rotation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getEulerYPR(yaw, pitch, roll);
        req->start_angle = yaw;

        while (!client->wait_for_service(1s)){
          if (!rclcpp::ok()){
            err("Interrupted while waiting for the service. Exiting.");
            return -1.0;
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
      gate_id = min_index;
      double p_length = lengths[min_index];
      log("Shortest path obtained.");
      return p_length;
    }

    void follow_path(){
      if(!path){
        err("There is no path to follow.");
        return;
      }
      
      auto client_node = rclcpp::Node::make_shared("follow_path_client");
      follow_path_client = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(client_node,"follow_path");

      if (!follow_path_client->wait_for_action_server(1s)) {
        err("Action server not available after waiting");
        return;
      }

      auto goal_msg = nav2_msgs::action::FollowPath::Goal();
      goal_msg.path = *path;
      goal_msg.controller_id = "FollowPath";
      log("Sending goal");

      auto goal_handle_future = follow_path_client->async_send_goal(goal_msg);
      if (rclcpp::spin_until_future_complete(client_node, goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS){
        err("send goal call failed.");
        return;
      }
      log("Goal sent.");
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::SharedPtr
        goal_handle = goal_handle_future.get();
      if (!goal_handle) {
        err("Goal was rejected by server");
        return;
      }
      auto result_future = follow_path_client->async_get_result(goal_handle);

      log("Waiting for result");
      if (rclcpp::spin_until_future_complete(client_node, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS){
        err("get result call failed");
        return;
      }

      rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult
        wrapped_result = result_future.get();

      switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          log("Goal reached.");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          err("Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          err("Goal was canceled");
          return;
        default:
          err("Unknown result code");
          return;
      }
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
        auto t = obtain_transform_of_shelfino(i);
        if(!t.header.frame_id.empty()){
          transforms.push_back(t);
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
        debug(s.str());
      }
      
      return p;
    }
    geometry_msgs::msg::TransformStamped obtain_transform_of_shelfino(int shelfino_id){
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
      geometry_msgs::msg::TransformStamped t;
      std::string topic_name = "/shelfino"+std::to_string(shelfino_id)+"/transform";
      log("Looking for topic:"+topic_name);

      // this is necessary if we want to use qos
      std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::TransformStamped>>
        sub = this->create_subscription<geometry_msgs::msg::TransformStamped>
        (topic_name,qos,[](const std::shared_ptr<const geometry_msgs::msg::TransformStamped>){});

      bool obtained = rclcpp::wait_for_message(t,sub, this->get_node_options().context(), 1s);
      if(obtained){
        log("found transform for shelfino"+std::to_string(shelfino_id));
      }else{
        err("Transform for shelfino"+std::to_string(shelfino_id)+" not found");
      }

      return t;
    }

    rclcpp_action::GoalResponse handle_evacuation_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const roadmap_interfaces::action::Evacuate_Goal> goal){
        debug("Goal acepted");
        if(goal->path.poses.size()>0){
          path = std::make_shared<nav_msgs::msg::Path>(goal->path);
        }
        start_delay = goal->delay;
        
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const 
      std::shared_ptr<rclcpp_action::ServerGoalHandle<roadmap_interfaces::action::Evacuate>> goal_handle){
      log("Received signal to cancel evacuate");
      // I'm not actually doing anything, I have no idea how to wrap it around
      // the follow action server is bugged anyway and doesn't really work properly
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const 
      std::shared_ptr<rclcpp_action::ServerGoalHandle<roadmap_interfaces::action::Evacuate>> goal_handle){
      std::thread{std::bind(&RobotDriver::execute_action_evacuate, this, _1), goal_handle}.detach();
    }

    void execute_action_evacuate(const 
      std::shared_ptr<rclcpp_action::ServerGoalHandle<roadmap_interfaces::action::Evacuate>> goal_handle){
      log("Start executing ecavuation action");
      if(start_delay > 0.0){
        log("Sleep for "+std::to_string(start_delay)+" seconds...");
        auto sleep = 1000ms * start_delay;
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(sleep));
        log("Wake up.");
      }
      follow_path();
      log("Ended follow path action");
      auto result = std::make_shared<roadmap_interfaces::action::Evacuate_Result>();
      result->shelfino_id = robot_id;
      goal_handle->succeed(result);
      log("----------End.");
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