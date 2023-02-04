#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "roadmap_interfaces/srv/path_service.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

class RobotDriver : public rclcpp::Node
{
  public:
    RobotDriver()
    : Node("robot_driver"){}

    void init(){
      const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

      
      gates_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        GATES_TOPIC, qos, std::bind(&RobotDriver::set_gates, this, _1));
    
      
      robot_position_subscriber = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        ROBOT_POSITION_TOPIC, qos, std::bind(&RobotDriver::set_robot_position, this, _1));
      
      log("Ready.");
    }

  private:
    // width + tollerance to be sure
    const double ROBOT_WIDTH = 0.5 + 0.2;

    const std::string ROADMAP_SERVICE_NAME = "compute_path";
    const std::string GATES_TOPIC = "/gate_position";
    const std::string ROBOT_POSITION_TOPIC = "transform";
    
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gates_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr robot_position_subscriber;
    

    std::shared_ptr<geometry_msgs::msg::PoseArray> gates;
    std::shared_ptr<geometry_msgs::msg::TransformStamped> robot_position;

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

    void guide_to_gate(){
      if(!gates){
        err("Gates pointer is null");
        return;
      }
      if(!robot_position){
        err("The robot position is null");
        return;
      }

      auto length_of_path = [](const nav_msgs::msg::Path path){
        double length = 0.0;
        for(int i=1; i<path.poses.size(); i++){
          double dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
          double dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
          double dz = path.poses[i].pose.position.z - path.poses[i-1].pose.position.z;
          double dd = std::sqrt(std::pow(dx, 2.0)+std::pow(dy, 2.0)+std::pow(dz, 2.0));
          length += dd;
        }
        return length;
      };

      std::shared_ptr<rclcpp::Node> client_node = rclcpp::Node::make_shared("roadmap_client");
      auto client = client_node->create_client<roadmap_interfaces::srv::PathService>(ROADMAP_SERVICE_NAME);
      
      std::vector<nav_msgs::msg::Path> possible_paths;

      for(auto gate:gates->poses){

        auto req = std::make_shared<roadmap_interfaces::srv::PathService::Request>();

        req->start.x = robot_position->transform.translation.x;
        req->start.y = robot_position->transform.translation.y;
        req->start.z = robot_position->transform.translation.z;

        req->end.x = gate.position.x;
        req->end.y = gate.position.z;
        req->end.z = gate.position.z;

        req->minimum_path_width = ROBOT_WIDTH / 2.0;

        while (!client->wait_for_service(1s)){
          if (!rclcpp::ok()){
            err("Interrupted while waiting for the service. Exiting.");
            return;
          }
          log("service not available, waiting again...");
        }

        auto result = client->async_send_request(req);

        if (rclcpp::spin_until_future_complete(client_node, result) ==
          rclcpp::FutureReturnCode::SUCCESS){
          if(result.get()->result)
            possible_paths.push_back(result.get()->path);
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