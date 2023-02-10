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

    void log(std::string log){
      RCLCPP_INFO(this->get_logger(), log.c_str());
    }

    void err(std::string log){
      RCLCPP_ERROR(this->get_logger(), log.c_str());
    }

    void coordinate_evacuation(
      const std::shared_ptr<std_srvs::srv::Empty_Request> request,
      std::shared_ptr<std_srvs::srv::Empty_Response> response){
      log("Start evacuation.");
 
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

      std::shared_ptr<rclcpp::Node> client_node = rclcpp::Node::make_shared("evacuate_client");

      do{
        find_lengths();
        if(path_lengths.size()==0){
          err("No paths");
          break;
        }

        int min_shelfino = min_key(path_lengths);

        std::string shelfino_service = "/shelfino"+std::to_string(min_shelfino)+"/"+EVACUATE_SERVICE;
        auto client = client_node->create_client<std_srvs::srv::Empty>(shelfino_service);

        while (!client->wait_for_service(1s)){
          if (!rclcpp::ok()){
            err("Interrupted while waiting for the service "+shelfino_service);
            return;
          }
          log("service not available, waiting again...");
        }
        log("Requesting evacuate service from driver "+std::to_string(min_shelfino));
        auto r = std::make_shared<std_srvs::srv::Empty_Request>();
        auto result = client->async_send_request(r);

        if (rclcpp::spin_until_future_complete(client_node, result) == rclcpp::FutureReturnCode::SUCCESS){
          log("Evacuation compleated");
          path_lengths.erase(min_shelfino);
          auto remover = client_node->create_client<gazebo_msgs::srv::DeleteEntity>(GAZEBO_DELETE_SERVICE);
          while (!remover->wait_for_service(1s)){
            if (!rclcpp::ok()){
              err("Interrupted while waiting for the service "+GAZEBO_DELETE_SERVICE);
              return;
            }
            log("service not available, waiting again...");
          }
          auto remove_request = std::make_shared<gazebo_msgs::srv::DeleteEntity_Request>();
          remove_request->name = "shelfino"+std::to_string(min_shelfino);
          auto remove_result = remover->async_send_request(remove_request);
          if (rclcpp::spin_until_future_complete(client_node, remove_result) == rclcpp::FutureReturnCode::SUCCESS){
            log("Shelfino"+std::to_string(min_shelfino)+" removed.");
          }else{
            err("Shelfino"+std::to_string(min_shelfino)+" NOT removed.");
          }

        }else{
          err("Failed to call service "+FIND_BEST_PATH_SERVICE);
        }
        
      }while(path_lengths.size()>0);
    }

    void find_lengths(){
      log("Find minimum path lengths");
      path_lengths.clear();
      std::shared_ptr<rclcpp::Node> client_node = rclcpp::Node::make_shared("coordinator_client_node");

      for(int i = 0; i<=N_ROBOTS; i++){
        std::string service_path = "/shelfino"+std::to_string(i)+"/"+FIND_BEST_PATH_SERVICE;
        auto client = client_node->create_client<roadmap_interfaces::srv::DriverService>(service_path);

        if(!client->wait_for_service(2s)){
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
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto pub = std::make_shared<Coordinator>();
  rclcpp::spin(pub);
  rclcpp::shutdown();
  
  return 0;
}