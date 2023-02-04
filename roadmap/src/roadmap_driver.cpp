#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"


/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
using namespace std::chrono_literals;

class RobotDriver : public rclcpp::Node
{
  public:
    RobotDriver()
    : Node("robot_driver"), ns("")
    {
      
    }

    void init(){
      
      log("Ready.");
    }

  private:
    std::string ns;


    void log(std::string log){
      RCLCPP_INFO(this->get_logger(), log.c_str());
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