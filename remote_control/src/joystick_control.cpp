#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mutex>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "joystick.hh"

using namespace std::chrono_literals;

rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;

std::mutex accMTX;
double acc;
std::mutex decMTX;
double dec;
std::mutex steerMTX;
double steer;
std::mutex terminateMTX;
bool terminate;

double V_MAX_LIMIT = 1;
double O_MAX_LIMIT = 1.5;

// ros2 run remote_control joystick_control shelfino2 --remap /shelfino2/cmd_vel:=/cmd_vel

std::string robot_id_topic;

/**
 * @brief ROS2 node to send velocities commands to the **cmd_vel** topic using the keyboard
 * 
 */
class CMDPublisher : public rclcpp::Node
{
  public:
    /**
     * @brief Construct a new CMDPublisher object
     * 
     */
    CMDPublisher()
    : Node("remote_control")
    {
      client_ = this->create_client<std_srvs::srv::SetBool>(robot_id_topic+"/power");
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }
      request->data = true;
      client_->async_send_request(request);

      publisher_ = rclcpp::Node::create_publisher<geometry_msgs::msg::Twist>(robot_id_topic+"/cmd_vel", 10);
      last = std::chrono::system_clock::now();
      timer_ = this->create_wall_timer(
      100ms, std::bind(&CMDPublisher::timer_callback, this));
    }

    private:
    /// @brief Method callback to publish the velocities command from the keyboard input.
    void timer_callback()
    {
      geometry_msgs::msg::Twist msg;
      auto curr = std::chrono::system_clock::now();
      std::chrono::duration<double> diff = curr - last;
      double dT = diff.count();
      last = curr;

      double filter = 0.7;
      double v = 0;
      double omega = 0;
      
      {
        std::unique_lock<std::mutex> lock(decMTX);
        v = -dec;
        lock.unlock();
      }
      {
        std::unique_lock<std::mutex> lock(accMTX);
        if(v == 0) v = acc;
        else if(v != 0 && acc != 0) v = 0;
        lock.unlock();
      }
      {
        std::unique_lock<std::mutex> lock(steerMTX);
        omega = -steer;
        lock.unlock();
      }

      v_c = (1-filter)*v*V_MAX_LIMIT + filter*v_c;
      omega_c = (1-filter)*omega*O_MAX_LIMIT + filter*omega_c;

      msg.linear.x = v_c;
      msg.angular.z = omega_c;

      publisher_->publish(msg);
      
      {
        std::unique_lock<std::mutex> lock(terminateMTX);
        if(terminate){
          msg.linear.x = 0.0;
          msg.angular.z = 0.0;
          publisher_->publish(msg);
          auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
          request->data = false;
          client_->async_send_request(request);
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          rclcpp::shutdown();
        }
        lock.unlock();
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;
    double v_c = 0.0, omega_c = 0.0, V_MAX = 0.0, O_MAX = 0.0;
    std::chrono::system_clock::time_point last;
};

/// @brief Function that checks if there is new data available from the keyboard without having to wait for the return key.
void joy()
{
  // Create an instance of Joystick
  Joystick joystick("/dev/input/js1");

  // Ensure that it was found and that we can use it
  if (!joystick.isFound())
  {
    std::cout << "Could not connect to joystick.\n";
    exit(1);
  }

  bool stop_flag = false;

  while (!stop_flag)
  {
    // Restrict rate
    usleep(1000);

    // Attempt to sample an event from the joystick
    JoystickEvent event;
    if (joystick.sample(&event))
    {
      if (event.isButton())
      {
        // printf("Button %u is %s\n",
        //   event.number,
        //   event.value == 0 ? "up" : "down");
        switch( event.number ){
          case 1:
            if( event.value == 1 ){
              std::unique_lock<std::mutex> lock(terminateMTX);
              terminate = true;
              lock.unlock();
              stop_flag = true;
            }
            break;
        }
      } 
      else if (event.isAxis())
      {
        // printf("Axis %u is at position %d\n", event.number, event.value);
        switch( event.number ){
          case 5: 
          {
            std::unique_lock<std::mutex> lock(accMTX);
            acc = (event.value + 32767) / 65534.0;
            lock.unlock();
            break;
          }
          case 2:
          {
            std::unique_lock<std::mutex> lock(decMTX);
            dec = (event.value + 32767) / 65534.0;
            lock.unlock();
            break;
          }
          case 0:
          {
            std::unique_lock<std::mutex> lock(steerMTX);
            steer = event.value / 32767.0;
            lock.unlock();
            break;
          }
        }
      }
    }
  }
}

/**
 * @brief Function to initialize the ROS node.
 * 
 * @param argc Number of arguments passed from the command line.
 * @param argv Array of arguments passed from the command line.
 */
void start_ros_node(int argc, char * argv[])
{
  if(argc < 2)  robot_id_topic = "";
  else          robot_id_topic = argv[1];
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CMDPublisher>());
}


/**
 * @brief Main function that start two separete threads for the ROS node and the keyboard listener.
 * 
 * @param argc Number of arguments passed from the command line.
 * @param argv Array of arguments passed from the command line.
 */
int main(int argc, char * argv[])
{
  std::unique_lock<std::mutex> lock(terminateMTX);
  terminate = false;
  lock.unlock();

  std::thread remote_control(start_ros_node, argc, argv);
  std::thread joystick_listener(joy);

  remote_control.join();
  joystick_listener.join();

  return 0;
}

