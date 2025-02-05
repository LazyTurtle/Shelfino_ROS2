#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/wait_for_message.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "roadmap_interfaces/srv/driver_service.hpp"
#include "roadmap_interfaces/action/evacuate.hpp"
#include "gazebo_msgs/srv/delete_model.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

class Coordinator : public rclcpp::Node
{
  public:

    Coordinator()
    : Node("robot_coordinator"){
      unsafe_coordinator_service = this->create_service<std_srvs::srv::Empty>(
        UNSAFE_COORDINATOR_SERVICE, std::bind(&Coordinator::coordinate_unsafe_ecacuation, this, _1, _2));
      safe_coordinator_service = this->create_service<std_srvs::srv::Empty>(
        SAFE_COORDINATOR_SERVICE, std::bind(&Coordinator::coordinate_safe_evacuation, this, _1, _2));
      
      number_of_gates = get_number_of_gates();
      log("Ready.");
    }

  private:
    const std::string UNSAFE_COORDINATOR_SERVICE = "unsafe_coordinate_evacuation";
    const std::string SAFE_COORDINATOR_SERVICE = "safe_coordinate_evacuation";
    const std::string FIND_BEST_PATH_SERVICE = "find_best_path";
    const std::string EVACUATE_SERVICE = "evacuate";
    const std::string GAZEBO_DELETE_SERVICE = "/delete_entity";
    const int N_ROBOTS = 3;
    const double SAFE_DISTANCE = 1.5; // in meters
    const double ROBOT_SPEED = 0.3; // in meters/s

    int number_of_gates;

    rclcpp::TimerBase::SharedPtr timer;
    std::vector<rclcpp::TimerBase::SharedPtr> timers;
    std::map<int,double> path_lengths;
    std::map<int,int> shelfino_gate_assignment;

    std::vector<rclcpp_action::Client<roadmap_interfaces::action::Evacuate>::SharedPtr> action_clients;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr safe_coordinator_service, unsafe_coordinator_service;

    void debug(std::string log){
      RCLCPP_DEBUG(this->get_logger(), log.c_str());
    }

    void log(std::string log){
      RCLCPP_INFO(this->get_logger(), log.c_str());
    }

    void err(std::string log){
      RCLCPP_ERROR(this->get_logger(), log.c_str());
    }

    int get_number_of_gates(){
      int n = -1;
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
      log("looking for the number of gates.");
      geometry_msgs::msg::PoseArray gates;
      std::string topic_name = "/gate_position";
      debug("Looking for topic:"+topic_name);

      // this is necessary if we want to use qos
      auto sub = this->create_subscription<geometry_msgs::msg::PoseArray>
        (topic_name,qos,[](const std::shared_ptr<const geometry_msgs::msg::PoseArray>){});

      bool obtained = rclcpp::wait_for_message(gates,sub, this->get_node_options().context());
      if(obtained){
        debug("Gates found.");
        n = gates.poses.size();
      }else{
        err("Error upon waiting for a message about the gates");
      }
      log("Number of gates "+std::to_string(n));
      return n;
    }

    void coordinate_unsafe_ecacuation(
      const std::shared_ptr<std_srvs::srv::Empty_Request> request,
      std::shared_ptr<std_srvs::srv::Empty_Response> response){
      auto find_all_robots_assigned_to_gate = [this](int gate_id){
        std::vector<int> v;
        for(auto const& pair:shelfino_gate_assignment){
          if(pair.second == gate_id)
            v.push_back(pair.first);
        }
        return v;
      };
      action_clients.clear();
      find_lengths(false);
      auto client_node = rclcpp::Node::make_shared("unsafe_evacuation");
      log("Start coordinating for each gate");
      for(int i=0; i<number_of_gates; i++){
        log("Looking at the line for gate "+std::to_string(i));
        std::vector<int>robots_to_this_gate = find_all_robots_assigned_to_gate(i);
        std::sort(robots_to_this_gate.begin(), robots_to_this_gate.end(),
          [this](int i, int j){return path_lengths[i]<path_lengths[j];});

        for(int i=0;i<robots_to_this_gate.size();i++){
          log("Looking for shelfino "+std::to_string(robots_to_this_gate[i]));
          std::string address = "/shelfino"+std::to_string(robots_to_this_gate[i])+"/action_evacuate";
          auto unsafe_evacuation_client = rclcpp_action::create_client
            <roadmap_interfaces::action::Evacuate>(this, address);
          action_clients.push_back(unsafe_evacuation_client);
          
          if(!unsafe_evacuation_client->wait_for_action_server(1s)){
            err("Action server not found for "+std::to_string(robots_to_this_gate[i]));
            continue;
          }
          auto goal = roadmap_interfaces::action::Evacuate_Goal();
          if(i > 0){
            double path_difference = 
              path_lengths[robots_to_this_gate[i]] - path_lengths[robots_to_this_gate[i-1]];
            double path_to_safe_distance = SAFE_DISTANCE - path_difference;
            if(path_to_safe_distance > 0.0){
              path_lengths[robots_to_this_gate[i]] += path_to_safe_distance;
              double time_to_wait = (path_to_safe_distance / ROBOT_SPEED);
              goal.delay = time_to_wait;
            }
          }
          log("Sending goal.");
          auto goal_options = rclcpp_action::Client<roadmap_interfaces::action::Evacuate>::SendGoalOptions();
          goal_options.result_callback = std::bind(&Coordinator::ending_callback, this, _1);
          unsafe_evacuation_client->async_send_goal(goal, goal_options);
        }
      }
    }

    void ending_callback(const 
      rclcpp_action::ClientGoalHandle<roadmap_interfaces::action::Evacuate>::WrappedResult& result){
      log("Action evacuate completed.");
      delete_shelfino(result.result->shelfino_id);
      shutdown_shelfino_transform(result.result->shelfino_id);
    }

    void coordinate_safe_evacuation(
      const std::shared_ptr<std_srvs::srv::Empty_Request> request,
      std::shared_ptr<std_srvs::srv::Empty_Response> response){
      log("Start coordinated evacuation.");
 
      auto min_key = [](const std::map<int,double>& map){
        double min_value = std::numeric_limits<double>().infinity();
        int key;
        for(auto pair:map){
          if(min_value>pair.second){
            min_value = pair.second;
            key = pair.first;
          }
        }
        return key;
      };

      find_lengths();
      while(path_lengths.size()>0){
        int min_shelfino = min_key(path_lengths);

        evacuate_shelfino(min_shelfino);

        find_lengths();
      }
      log("Coordination finished.");
    }

    void evacuate_shelfino(int shelfino_id){
      log("Evacuate shelfino"+std::to_string(shelfino_id));
      std::shared_ptr<rclcpp::Node> client_node = rclcpp::Node::make_shared("coordinator_evacuate_client");
      std::string shelfino_service = "/shelfino"+std::to_string(shelfino_id)+"/"+EVACUATE_SERVICE;
      auto client = client_node->create_client<std_srvs::srv::Empty>(shelfino_service);

      while (!client->wait_for_service(1s)){
        if (!rclcpp::ok()){
          err("Interrupted while waiting for the service "+shelfino_service);
          return;
        }
        err("service not available, waiting again...");
      }

      debug("Requesting evacuate service from driver "+std::to_string(shelfino_id));
      auto r = std::make_shared<std_srvs::srv::Empty_Request>();
      auto result = client->async_send_request(r);
      if (rclcpp::spin_until_future_complete(client_node, result) == rclcpp::FutureReturnCode::SUCCESS){
        log("Evacuation compleated");
        delete_shelfino(shelfino_id);
        shutdown_shelfino_transform(shelfino_id);
      }else{
        err("Failed to call service "+FIND_BEST_PATH_SERVICE);
      }
    }

    void find_lengths(bool consider_other_robots = true){
      log("Find minimum path lengths");
      path_lengths.clear();
      std::shared_ptr<rclcpp::Node> client_node = rclcpp::Node::make_shared("coordinator_find_path_client");

      for(int i = 0; i<=N_ROBOTS; i++){
        std::string service_path = "/shelfino"+std::to_string(i)+"/"+FIND_BEST_PATH_SERVICE;
        auto client = client_node->create_client<roadmap_interfaces::srv::DriverService>(service_path);

        if(!client->wait_for_service(1s)){
          err("Not found for "+service_path);
          continue;
        }
        log("Requesting path length from driver "+std::to_string(i));
        auto r = std::make_shared<roadmap_interfaces::srv::DriverService_Request>();
        r->look_for_robots = consider_other_robots;
        auto result = client->async_send_request(r);

        if (rclcpp::spin_until_future_complete(client_node, result) == rclcpp::FutureReturnCode::SUCCESS){
          auto res = result.get();
          if(res->result && res->length>0.0){
            path_lengths[i] = res->length;
            shelfino_gate_assignment[i] = res->gate_id;
            log("Path length to "+std::to_string(res->gate_id)+" gate obtained: "+std::to_string(res->length));
          }
          else{
            err("There was no availabe path.");
          }
        }else{
          err("Failed to call service "+FIND_BEST_PATH_SERVICE);
        }
      }
    }

    bool delete_shelfino(int shelfino_id){
      log("Delete shelfino "+std::to_string(shelfino_id));
      bool delete_result = false;
      path_lengths.erase(shelfino_id);
      auto shelfino_remover_node = rclcpp::Node::make_shared("shelfino_remover_node");
      auto shelfino_remover = shelfino_remover_node->create_client<gazebo_msgs::srv::DeleteEntity>(GAZEBO_DELETE_SERVICE);
      debug("waiting for the service...");
      while (!shelfino_remover->wait_for_service(1s)){
        if (!rclcpp::ok()){
          err("Interrupted while waiting for the service "+GAZEBO_DELETE_SERVICE);
          return delete_result;
        }
        debug("service not available, waiting again...");
      }
      auto remove_request = std::make_shared<gazebo_msgs::srv::DeleteEntity_Request>();
      std::string shelfino_name = "shelfino"+std::to_string(shelfino_id);
      remove_request->name = shelfino_name.c_str();
      debug("Trying to delete "+shelfino_name);
      auto remove_result = shelfino_remover->async_send_request(remove_request);
      if (rclcpp::spin_until_future_complete(shelfino_remover_node, remove_result) == rclcpp::FutureReturnCode::SUCCESS){
        auto result = remove_result.get();
        if(result->success){
          log(result->status_message);
          delete_result = true;
        }else{
          err(result->status_message);
        }
      }else{
        err("Service call failed.");
      }
      return delete_result;
    }

    bool shutdown_shelfino_transform(int shelfino_id){
      std::string TRANSFORM_SERVICE = "/shelfino"+std::to_string(shelfino_id)+"/shutdown";
      bool shutdown_result = false;

      auto transform_remover_node = rclcpp::Node::make_shared("transform_remover_node");
      auto transform_remover = transform_remover_node->create_client<std_srvs::srv::Empty>(TRANSFORM_SERVICE);

      while (!transform_remover->wait_for_service(1s)){
        if (!rclcpp::ok()){
          err("Interrupted while waiting for the service "+TRANSFORM_SERVICE);
          return shutdown_result;
        }
        debug("service not available, waiting again...");
      }
      auto empty = std::make_shared<std_srvs::srv::Empty_Request>();
      auto empty_res = transform_remover->async_send_request(empty);
      if (rclcpp::spin_until_future_complete(transform_remover_node, empty_res) == rclcpp::FutureReturnCode::SUCCESS){
        log("Transform shutdown");
        shutdown_result = true;
      }else{
        err("Transform shutdown error.");
      }
      return shutdown_result;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto pub = std::make_shared<Coordinator>();
  rclcpp::spin(pub);
  rclcpp::shutdown();
  
  return 0;
}