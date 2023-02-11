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
      coordinator_service = this->create_service<std_srvs::srv::Empty>(
        COORDINATOR_SERVICE, std::bind(&Coordinator::coordinate_evacuation, this, _1, _2));
      log("Ready.");
    }

  private:
    const std::string COORDINATOR_SERVICE = "coordinate_evacuation";
    const std::string FIND_BEST_PATH_SERVICE = "find_best_path";
    const std::string EVACUATE_SERVICE = "evacuate";
    const std::string GAZEBO_DELETE_SERVICE = "/delete_entity";

    rclcpp::TimerBase::SharedPtr timer;
    const int N_ROBOTS = 3;
    std::map<int,double> path_lengths;
    bool bCoordinate = false;
    
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr coordinator_service;

    void debug(std::string log){
      RCLCPP_DEBUG(this->get_logger(), log.c_str());
    }

    void log(std::string log){
      RCLCPP_INFO(this->get_logger(), log.c_str());
    }

    void err(std::string log){
      RCLCPP_ERROR(this->get_logger(), log.c_str());
    }

    void coordinate_evacuation(
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
        log("service not available, waiting again...");
      }

      log("Requesting evacuate service from driver "+std::to_string(shelfino_id));
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

    void find_lengths(){
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
        auto result = client->async_send_request(r);

        if (rclcpp::spin_until_future_complete(client_node, result) == rclcpp::FutureReturnCode::SUCCESS){
          auto res = result.get();
          if(res->result && res->length>0.0){
            path_lengths[i] = res->length;
            log("Path length obtained: "+std::to_string(res->length));
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