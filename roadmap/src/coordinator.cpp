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

class Coordinator : public rclcpp::Node
{
  public:
    std::shared_ptr<nav_msgs::msg::Path> path;
    double path_length;


    Coordinator()
    : Node("robot_coordinator"){}

    void init(){
    }

  private:

    const int N_ROBOTS = 3;
    
    const std::string FIND_BEST_PATH_SERVICE = "find_best_path";


    void log(std::string log){
      RCLCPP_INFO(this->get_logger(), log.c_str());
    }

    void err(std::string log){
      RCLCPP_ERROR(this->get_logger(), log.c_str());
    }

    void evacuation(){
      std::vector<double> path_lengths;
      for(int i = 0; i<=N_ROBOTS; i++){
        
      }
    }
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto pub = std::make_shared<Coordinator>();
  pub->init();
  rclcpp::spin(pub);
  rclcpp::shutdown();
  
  return 0;
}