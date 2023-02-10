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

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

class Coordinator : public rclcpp::Node
{
  public:

    Coordinator()
    : Node("robot_coordinator"){

    }

  private:

    const std::string FIND_BEST_PATH_SERVICE = "find_best_path";
    const std::string EVACUATE_SERVICE = "evacuate";

    rclcpp::TimerBase::SharedPtr timer;
    const int N_ROBOTS = 3;
    std::map<int,double> path_lengths;


    bool bCoordinate = false;


    void log(std::string log){
      RCLCPP_INFO(this->get_logger(), log.c_str());
    }

    void err(std::string log){
      RCLCPP_ERROR(this->get_logger(), log.c_str());
    }

    void coordination_check(){    
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
      log("Coordination check.");

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
        auto r = std::shared_ptr<std_srvs::srv::Empty_Request>();
        auto result = client->async_send_request(r);

        if (rclcpp::spin_until_future_complete(client_node, result) == rclcpp::FutureReturnCode::SUCCESS){
          log("Evacuation compleated");
          path_lengths.erase(min_shelfino);
        }else{
          err("Failed to call service "+FIND_BEST_PATH_SERVICE);
        }
        
      }while(path_lengths.size()>0);
    }

    void find_lengths(){
      path_lengths.clear();
      std::shared_ptr<rclcpp::Node> client_node = rclcpp::Node::make_shared("driver_client");

      for(int i = 0; i<=N_ROBOTS; i++){
        std::string service_path = "/shelfino"+std::to_string(i)+"/"+FIND_BEST_PATH_SERVICE;
        auto client = client_node->create_client<roadmap_interfaces::srv::DriverService>(service_path);

        while (!client->wait_for_service(1s)){
          if (!rclcpp::ok()){
            err("Interrupted while waiting for the service. Exiting.");
            return;
          }
          log("service not available, waiting again...");
        }
        log("Requesting path length from driver "+std::to_string(i));
        auto r = std::shared_ptr<roadmap_interfaces::srv::DriverService_Request>();
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