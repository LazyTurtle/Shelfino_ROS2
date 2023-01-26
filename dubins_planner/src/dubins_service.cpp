#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "dubins_planner_msgs/srv/dubins_planning.hpp"
#include "dubins_planner_msgs/srv/multi_point_dubins_planning.hpp"

#include "dubins.cpp"

#define PI 3.14159265

class DubinsCalculator : public rclcpp::Node
{
  public:
    DubinsCalculator():Node("server"){

      dubins_service = this->create_service<dubins_planner_msgs::srv::DubinsPlanning>
        ("dubins_calculator", 
        std::bind(&DubinsCalculator::calculate_dubins, this,
        std::placeholders::_1,
        std::placeholders::_2));

      multi_point_dubins_service = this->create_service<dubins_planner_msgs::srv::MultiPointDubinsPlanning>
        ("multi_points_dubins_calculator", 
        std::bind(&DubinsCalculator::calculate_mp_dubins, this,
        std::placeholders::_1,
        std::placeholders::_2));

        log_info("Ready.");

    }

  private:
    std::shared_ptr<rclcpp::Service<dubins_planner_msgs::srv::DubinsPlanning>> dubins_service;
    std::shared_ptr<rclcpp::Service<dubins_planner_msgs::srv::MultiPointDubinsPlanning>> multi_point_dubins_service;

    void calculate_dubins(
      const std::shared_ptr<dubins_planner_msgs::srv::DubinsPlanning::Request> request,
      const std::shared_ptr<dubins_planner_msgs::srv::DubinsPlanning::Response> response){
      
      log_info("Dubins calculator: request received... ");
      
      float x0 = request->start.point.x;
      float y0 = request->start.point.y;
      float th0 = PI*request->start.angle;

      float xf = request->end.point.x;
      float yf = request->end.point.y;
      float thf = PI*request->end.angle;

      float Kmax = request->kmax;

      Dubins_curve curve = dubins_shortest_path(x0, y0, th0, xf, yf, thf, Kmax);
      response->path = plot_dubins(curve);
      response->lenght = curve.L;
    }

    nav_msgs::msg::Path plot_dubins(Dubins_curve curve){
      float ret[((npts+1)*3)][2];
      Plot_arc_struct a1 = plot_arc(curve.a1);
      for(int r= 0; r<npts; r++){
        for(int c = 0; c<2; c++){
          cout<<a1.pts[r][c]<<"\t";
          
          ret[r][c] = a1.pts[r][c];
          
        }
        cout<<"\n\n";
          
      }

      Plot_arc_struct a2 = plot_arc(curve.a2);
      for(int r= 0; r<npts; r++){
        for(int c = 0; c<2; c++){
          cout<<a2.pts[r][c]<<"\t";
          ret[r+100][c] = a2.pts[r][c];
            
        }
        cout<<"\n\n";
      }
      Plot_arc_struct a3 = plot_arc(curve.a3);
      for(int r= 0; r<npts; r++){
        for(int c = 0; c<2; c++){
          cout<<a3.pts[r][c]<<"\t";
          ret[r+200][c] = a3.pts[r][c];
        }
        cout<<"\n\n";
      }
      cout<<"******************";
      for(int r= 0; r<(npts)*3; r++){
        for(int c = 0; c<2; c++){
          cout<<ret[r][c]<<"\t";
        }
        cout<<"\n";
      }

      nav_msgs::msg::Path path_msg;
      std::vector<geometry_msgs::msg::PoseStamped> poses_temp;
      path_msg.header.stamp = this->get_clock()->now();
      path_msg.header.frame_id = "map";
      geometry_msgs::msg::Pose pose_temp;
      geometry_msgs::msg::Point position_temp;
      geometry_msgs::msg::Quaternion quaternion_temp;
      geometry_msgs::msg::PoseStamped pose_stamped_temp;

      for(int i = 0; i<((npts)*3); i++) {
            
        position_temp.x = ret[i][0];
        
        position_temp.y = ret[i][1];
        position_temp.z = 0 ;

        quaternion_temp.x =  0.0;
        quaternion_temp.y =  0.0;
        quaternion_temp.z =  0.0;
        quaternion_temp.w =  0.0;

        if(i == ((npts)*3)-1){
          quaternion_temp.z =  0.9252115;
          quaternion_temp.w = -0.3794518;
        }

        pose_temp.position = position_temp;
        pose_temp.orientation = quaternion_temp;

        pose_stamped_temp.pose = pose_temp;
        pose_stamped_temp.header.stamp = this->get_clock()->now();
        pose_stamped_temp.header.frame_id = "";

        if (i <((npts)*3)-30 || i == ((npts)*3)-1){
          poses_temp.push_back(pose_stamped_temp);
        }

      }
      path_msg.poses = poses_temp;

      return path_msg;
    }

    Plot_arc_struct plot_arc(Dubins_arc arc){
      float pts[npts+1][2] = {};
      for(int j = 0; j <= npts; j++){
          float s = arc.L/npts * j;
          Circle_line ci;
          ci = circline(s, arc.x0, arc.y0, arc.th0, arc.k);
          float x = ci.x;
          float y = ci.y;
          pts[j][0] = x;
          pts[j][1] = y;
      }

      Plot_arc_struct a;
      for(int r= 0; r<npts; r++){
          for(int c = 0; c<2; c++){
              a.pts[r][c] = pts[r][c];
          }
          
      }  
      return a;
    }

    void calculate_mp_dubins(
      const std::shared_ptr<dubins_planner_msgs::srv::MultiPointDubinsPlanning::Request> request,
      const std::shared_ptr<dubins_planner_msgs::srv::MultiPointDubinsPlanning::Response> response){

        std::vector<geometry_msgs::msg::Point> waypoint_list = request->points;
        float final_angle = request->angle;
        float Kmax = request->kmax;
        int omega = request->komega;

        Dubins_curve minCurve;
        minCurve.L = std::numeric_limits<float>::max();
        std::vector<float>thetas(waypoint_list.size());

        std::vector<float>initial_angles;
        std::vector<float>final_angles;

        for(int i=0;i<omega;i++){
          float angle = ((2*PI)/omega)*i;
          initial_angles.push_back(angle);
        }

        if(final_angle<0.0){
          // the negative value indicates no preference
          // the angle is in [0,2π]
          for(int i=0;i<omega;i++){
            float angle = ((2*PI)/omega)*i;
            final_angles.push_back(angle);
          }
        }else{
          final_angles.push_back(final_angle);
        }

        for(auto start_angle : initial_angles){
          for(auto end_angle : final_angles){
            int n_wp = waypoint_list.size();
            Dubins_curve temp_curve = dubins_shortest_path(
              waypoint_list[n_wp-2].x, waypoint_list[n_wp-2].y, start_angle,
              waypoint_list[n_wp-1].x, waypoint_list[n_wp-1].y, end_angle,
              Kmax);
            
            if(temp_curve.L<minCurve.L){
              minCurve = temp_curve;
              thetas.end()[-1]=end_angle;
              thetas.end()[-2]=start_angle;
            }
          }
        }

        if(minCurve.L<0.0){
          log_err("Can't find a path.");
          return;
        }
        
        response->path = plot_dubins(minCurve);
        response->lenght = minCurve.L;

        for(int i = waypoint_list.size()-3; i>=0; i--){
          Dubins_curve temp_min_curve;
          temp_min_curve.L = std::numeric_limits<float>::max();

          for(auto start_angle : initial_angles){
            
            Dubins_curve temp_curve = dubins_shortest_path(
              waypoint_list[i].x, waypoint_list[i].y, start_angle,
              waypoint_list[i+1].x, waypoint_list[i+1].y, thetas[i+1],
              Kmax);
            
            if(temp_curve.L<temp_min_curve.L){
              temp_min_curve = temp_curve;
              thetas[i] = start_angle;
            }
          }

          response->lenght += temp_min_curve.L;
          nav_msgs::msg::Path temp_path = plot_dubins(temp_min_curve);
          response->path.poses.insert(
            response->path.poses.begin(), 
            temp_path.poses.begin(), temp_path.poses.end());
        }
        std::stringstream s;
        s << "Multi point dubins path calculation compleated.\n";
        s << "Lenght: "<<response->lenght<<"\n";
        log_info(s.str());
    }


    inline void log_info(const std::string log){
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Dubins calculator: "<<log);
    }

    inline void log_err(const std::string log){
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),"Dubins calculator: "<<log);
    }

    inline void log_warn(const std::string log){
      RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"),"Dubins calculator: "<<log);
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node=std::make_shared<DubinsCalculator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}