#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "dubins_planner_msgs/srv/dubins_planning.hpp"
#include "dubins_planner_msgs/srv/multi_point_dubins_planning.hpp"

#define PI 3.14159265

using namespace std::chrono_literals;

////////////////////////////////////////
///// Structures and Classes

struct DubinsArc{
  double 
    x0, y0, th0,
    xf, yf, thf,
    k, L = -1.0;
} typedef DubinsArc;

struct DubinsCurve{
  DubinsArc a1, a2, a3;
  double L = -1.0;
} typedef DubinsCurve;


////////////////////////////////////////
///// Node Classes

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
  
    void calculate_dubins(
      const std::shared_ptr<dubins_planner_msgs::srv::DubinsPlanning::Request> request,
      const std::shared_ptr<dubins_planner_msgs::srv::DubinsPlanning::Response> response){

      log_info("Dubins calculator: request received... ");
      
      double x0 = request->start.point.x;
      double y0 = request->start.point.y;
      double th0 = PI*request->start.angle;

      double xf = request->end.point.x;
      double yf = request->end.point.y;
      double thf = PI*request->end.angle;

      double Kmax = request->kmax;

      DubinsCurve curve;
      int curve_id;
      std::tie(curve_id, curve) = dubins_shortest_path(x0, y0, th0, xf, yf, thf, Kmax);

      std::vector<std::tuple<double, double>> points = getPlotPoints(curve);

      response->path = convert_to_path(points);
      response->lenght = curve.L;
    }

  void calculate_mp_dubins(
      const std::shared_ptr<dubins_planner_msgs::srv::MultiPointDubinsPlanning::Request> request,
      const std::shared_ptr<dubins_planner_msgs::srv::MultiPointDubinsPlanning::Response> response){

      std::vector<geometry_msgs::msg::Point> waypoint_list = request->points;
      double final_angle = request->angle;
      double Kmax = request->kmax;
      int omega = request->komega;

      log_info("Number of waypoints: "+std::to_string(waypoint_list.size()));

      DubinsCurve minCurve;
      minCurve.L = std::numeric_limits<double>::max();
      std::vector<double>thetas(waypoint_list.size(), 0);

      std::vector<double>initial_angles;
      std::vector<double>final_angles;

      for(int i=0;i<omega;i++){
        double angle = ((2*PI)/omega)*i;
        initial_angles.push_back(angle);
      }

      if(final_angle<0.0){
        // the negative value indicates no preference
        // the angle is in [0,2π]
        for(int i=0;i<omega;i++){
          double angle = ((2*PI)/omega)*i;
          final_angles.push_back(angle);
        }
      }else{
        final_angles.push_back(final_angle);
      }

      log_info("Start looking for the path from the last point to its previous one.");
      
      for(auto start_angle : initial_angles){
        for(auto end_angle : final_angles){
          DubinsCurve temp_curve;
          int temp_curve_id, n_wp = waypoint_list.size();
          std::tie(temp_curve_id, temp_curve) = dubins_shortest_path(
            waypoint_list[n_wp-2].x, waypoint_list[n_wp-2].y, start_angle,
            waypoint_list[n_wp-1].x, waypoint_list[n_wp-1].y, end_angle,
            Kmax);
          
          if(temp_curve_id>-1 && temp_curve.L<minCurve.L){
            minCurve = temp_curve;
            thetas.end()[-1]=end_angle;
            thetas.end()[-2]=start_angle;
          }
        }
      }
      {
        std::ostringstream s;
        if(minCurve.L<0.0){
          s<<"Can't find a path from final point x:"<<waypoint_list.back().x<<" y:"<<waypoint_list.back().y<<" to its previous one.";
          log_err(s.str());
          return;
        }else{
          s<<"Path from final point x:"<<waypoint_list.back().x<<" y:"<<waypoint_list.back().y<<" to its previous one found.";
          log_info(s.str());
        }
      }
      
      int refinements = request->refinements;
      for(int cycle = 0; cycle<refinements+1; cycle++){
        log_info("Refinement cycle "+std::to_string(cycle));
        double base = 1.5*(2.0/omega);
        double bound_ratio = std::pow(base, cycle);
        double bound = PI*bound_ratio;
        double diff = 2*bound/omega;
        // let's start from the second last one
        for(int i = waypoint_list.size()-2; i>=0; i--){
          DubinsCurve temp_min_curve;
          temp_min_curve.L = std::numeric_limits<double>::max();
          double previous_theta = thetas[i];

          for(double j = (-bound); j<=bound+1e-5; j+=diff){
            double local_angle = previous_theta+j;
            DubinsCurve temp_curve;
            int temp_curve_id;

            std::tie(temp_curve_id, temp_curve) = dubins_shortest_path(
              waypoint_list[i].x, waypoint_list[i].y, local_angle,
              waypoint_list[i+1].x, waypoint_list[i+1].y, thetas[i+1],
              Kmax);
            
            if(temp_curve_id>-1 && temp_curve.L<temp_min_curve.L){
              temp_min_curve = temp_curve;
              thetas[i] = local_angle;
            }
          }
          std::ostringstream s;
          s<<"Found path for "<<i<<"-th point.";
          log_info(s.str());
        }
      }

      log_info("Creation final path.");

      for(int i = 0; i<thetas.size()-1; i++){
        DubinsCurve temp_curve;
        int temp_curve_id;

        std::tie(temp_curve_id, temp_curve) = dubins_shortest_path(
          waypoint_list[i].x, waypoint_list[i].y, thetas[i],
          waypoint_list[i+1].x, waypoint_list[i+1].y, thetas[i+1],
          Kmax);
        
        nav_msgs::msg::Path temp_path = convert_to_path(temp_curve);

        response->path.poses.insert(
          response->path.poses.end(), temp_path.poses.begin(), temp_path.poses.end());
        response->lenght += temp_curve.L;
      }
      {
        std::stringstream s;
        s << "Multi point dubins path calculation compleated.\n";
        s << "Poses: "<<response->path.poses.size()<<"\n";
        s << "Lenght: "<<response->lenght<<"\n";
        log_info(s.str());
      }
    }

  private:
    std::shared_ptr<rclcpp::Service<dubins_planner_msgs::srv::DubinsPlanning>> dubins_service;
    std::shared_ptr<rclcpp::Service<dubins_planner_msgs::srv::MultiPointDubinsPlanning>> multi_point_dubins_service;

    nav_msgs::msg::Path convert_to_path(
      const DubinsCurve& curve, const int nPoints = 5){
        return convert_to_path(getPlotPoints(curve, nPoints));
    }

    nav_msgs::msg::Path convert_to_path(
      const std::vector<std::tuple<double, double>>& points){
      
      nav_msgs::msg::Path path;
      path.header.stamp = this->get_clock()->now();
      path.header.frame_id = "map";

      for(auto point : points){
        geometry_msgs::msg::PoseStamped temp;
        temp.header.stamp = this->get_clock()->now();
        temp.header.frame_id = "";
        temp.pose.position.x = std::get<0>(point);
        temp.pose.position.y = std::get<1>(point);
        temp.pose.position.z = 0 ;
        temp.pose.orientation.x = 0;
        temp.pose.orientation.y = 0;
        temp.pose.orientation.z = 0;
        temp.pose.orientation.w = 1;
        path.poses.push_back(temp);
      }
      return path;

    }

    ////////////////////////////////////////
    ///// Helper functions

    static double sinc(const double t){
      double s= (abs(t)<0.002) ? (1 - pow(t,2)/6 * (1 - pow(t,2)/20)) : sin(t)/t;
      return s;
    }

    // Normalize an angle in range [0,2*pi)
    static double mod2pi(const double angle){
      double moduled_angle = angle;
      while(moduled_angle<0){
        moduled_angle += 2*PI;
      }
      while(moduled_angle>=2*PI){
        moduled_angle -= 2*PI;
      }
      return moduled_angle;
    }

    // Normalize an angle in range [-pi,pi)
    static double rangeSymm(const double angle){
      double moduled_angle = angle;
      while(moduled_angle<-PI){
        moduled_angle += 2*PI;
      }
      while(moduled_angle>=PI){
        moduled_angle -= 2*PI;
      }
      return moduled_angle;
    }

    static bool check_dubins(
      const double s1, const double k0,
      const double s2, const double k1,
      const double s3, const double k2,
      const double th0,const double thf){

        const double x0 = -1.0;
        const double y0 = 0.0;
        const double xf = 1.0;
        const double yf = 0.0;

        const double eq1 = 
          x0 + 
          s1 * sinc((1./2.) * k0 * s1) * cos(th0 + (1./2.) * k0 * s1) + 
          s2 * sinc((1./2.) * k1 * s2) * cos(th0 + k0 * s1 + (1./2.) * k1 * s2) +
          s3 * sinc((1./2.) * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + (1./2.) * k2 * s3) - xf;

        const double eq2 = 
          y0 + 
          s1 * sinc((1/2.) * k0 * s1) * sin(th0 + (1/2.) * k0 * s1) + 
          s2 * sinc((1/2.) * k1 * s2) * sin(th0 + k0 * s1 + (1/2.) * k1 * s2) + 
          s3 * sinc((1/2.) * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + (1/2.) * k2 * s3) - yf;

        const double eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);

        const bool Lpos = (s1 > 0) || (s2 > 0) || (s3 > 0);
        const double epsilon = sqrt(eq1*eq1+eq2*eq2+eq3*eq3);
        const bool result = (epsilon < 1.e-6) && Lpos;
        return result;
      }

    // Scale the input problem to standard form (x0: -1, y0: 0, xf: 1, yf: 0)
    static std::tuple<double, double, double, double> scaleToStandard(
      const double x0, const double y0, const double th0, 
      const double xf, const double yf, const double thf, 
      const double Kmax){
        const double dx = xf - x0;
        const double dy = yf - y0;
        const double phi = atan2(dy,dx);
        const double lambda = hypot(dx,dy)/2.0;

        const double scaled_th0 = mod2pi(th0 - phi);
        const double scaled_th1 = mod2pi(thf - phi);
        const double scaled_Kmax = Kmax*lambda;
        auto result = std::make_tuple(scaled_th0, scaled_th1, scaled_Kmax, lambda);
        return result;
      }

    static std::tuple<double, double, double> scaleFromStandard(
      const double lambda, const double sc_s1, const double sc_s2, const double sc_s3){

        const double s1 = sc_s1 * lambda;
        const double s2 = sc_s2 * lambda;
        const double s3 = sc_s3 * lambda;
        auto result = std::make_tuple(s1,s2,s3);
        return result;
      }

    static std::tuple<double,double,double> circline(
      const double s, const double x0, const double y0, const double th0, const double k){

        const double x = x0 + s * sinc(k * s / 2.0) * cos(th0 + k * s / 2);
        const double y = y0 + s * sinc(k * s / 2.0) * sin(th0 + k * s / 2);
        const double th = mod2pi(th0 + k * s);
        return std::make_tuple(x,y,th);
    }

    const DubinsArc getArc(
      const double x0, const double y0, const double th0, const double k, const double L){

        DubinsArc arc;
        arc.x0 = x0;
        arc.y0 = y0;
        arc.th0 = th0;
        arc.k = k;
        arc.L = L;
        std::tie(arc.xf, arc.yf, arc.thf) = circline(L,x0,y0,th0,k);
        return arc;
      }

    const DubinsCurve getCurve(
      const double x0, const double y0, const double th0,
      const double s1, const double s2, const double s3,
      const double k0, const double k1, const double k2){

        DubinsCurve curve;
        curve.a1 = getArc(x0, y0, th0, k0, s1);
        curve.a2 = getArc(curve.a1.xf, curve.a1.yf, curve.a1.thf, k1, s2);
        curve.a3 = getArc(curve.a2.xf, curve.a2.yf, curve.a2.thf, k2, s3);
        curve.L = curve.a1.L + curve.a2.L + curve.a3.L;
        return curve;
    }

    std::vector<std::tuple<double,double>> getPlotPoints(
      const DubinsArc arc, const int nPoints=10){

      std::vector<std::tuple<double,double>> points;
      for(int i = 0; i<nPoints; i++){
        double s = arc.L/nPoints * i;
        auto point = circline(s,arc.x0,arc.y0,arc.th0,arc.k);
        points.push_back(std::make_tuple(std::get<0>(point),std::get<1>(point)));
      }
      return points;
    }

    std::vector<std::tuple<double,double>> getPlotPoints(
      const DubinsCurve& curve, const int nPoints=5){

        std::vector<std::tuple<double,double>> points;
        auto arc1Points = getPlotPoints(curve.a1, nPoints);
        auto arc2Points = getPlotPoints(curve.a2, nPoints);
        auto arc3Points = getPlotPoints(curve.a3, nPoints);

        points.insert(points.end(),arc1Points.begin(),arc1Points.end());
        points.insert(points.end(),arc2Points.begin(),arc2Points.end());
        points.insert(points.end(),arc3Points.begin(),arc3Points.end());

        return points;
    }

    ////////////////////////////////////////
    ///// DUBINS SOLUTIONS

    static std::tuple<bool,double,double,double> LSL(
      const double sc_th0, const double sc_thf, const double sc_Kmax){
        double sc_s1, sc_s2, sc_s3;
        bool ok = false;

        const double invK = 1 / sc_Kmax;
        const double C = cos(sc_thf) - cos(sc_th0);
        const double S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
        const double temp1 = atan2(C, S);
        const double temp2 = 2 + 4 * pow(sc_Kmax,2) - 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));

        sc_s1 = invK * mod2pi(temp1 - sc_th0);
        if(temp2 < 0){
          ok = false;
          sc_s1 = 0; sc_s2 = 0; sc_s3 = 0;
        }else{
          ok = true;
          sc_s2 = invK * sqrt(temp2);
          sc_s3 = invK * mod2pi(sc_thf - temp1);
        }

        auto result = std::make_tuple(ok, sc_s1, sc_s2, sc_s3);
        return result;
      }

    static std::tuple<bool,double,double,double> RSR(
      const double sc_th0, const double sc_thf, const double sc_Kmax){
        double sc_s1, sc_s2, sc_s3;
        bool ok = false;

        const double invK = 1 / sc_Kmax;
        const double C = cos(sc_th0) - cos(sc_thf);
        const double S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
        const double temp1 = atan2(C, S);
        const double temp2 = 2 + 4 * pow(sc_Kmax,2) - 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));

        sc_s1 = invK * mod2pi(sc_th0 - temp1);
        if(temp2 < 0){
          ok = false;
          sc_s1 = 0; sc_s2 = 0; sc_s3 = 0;
        }else{
          ok = true;
          sc_s2 = invK * sqrt(temp2);
          sc_s3 = invK * mod2pi(temp1 - sc_thf);
        }

        auto result = std::make_tuple(ok, sc_s1, sc_s2, sc_s3);
        return result;
      }

    static std::tuple<bool,double,double,double> LSR(
      const double sc_th0, const double sc_thf, const double sc_Kmax){
        double sc_s1, sc_s2, sc_s3;
        bool ok = false;

        const double invK = 1 / sc_Kmax;
        const double C = cos(sc_th0) + cos(sc_thf);
        const double S = 2 * sc_Kmax + sin(sc_th0) + sin(sc_thf);
        const double temp1 = atan2(-C, S);
        const double temp3 = 4 * pow(sc_Kmax,2) - 2 + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));

        if(temp3 < 0){
          ok = false;
          sc_s1 = 0; sc_s2 = 0; sc_s3 = 0;
        }else{
          ok = true;
          sc_s2 = invK * sqrt(temp3);
          const double temp2 = -atan2(-2, sc_s2 * sc_Kmax);
          sc_s1 = invK * mod2pi(temp1 + temp2 - sc_th0);
          sc_s3 = invK * mod2pi(temp1 + temp2 - sc_thf);
        }

        auto result = std::make_tuple(ok, sc_s1, sc_s2, sc_s3);
        return result;
      }

    static std::tuple<bool,double,double,double> RSL(
      const double sc_th0, const double sc_thf, const double sc_Kmax){
        double sc_s1, sc_s2, sc_s3;
        bool ok = false;

        const double invK = 1 / sc_Kmax;
        const double C = cos(sc_th0) + cos(sc_thf);
        const double S = 2 * sc_Kmax - sin(sc_th0) - sin(sc_thf);
        const double temp1 = atan2(C, S);
        const double temp3 = 4 * pow(sc_Kmax,2) - 2 + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));

        if(temp3 < 0){
          ok = false;
          sc_s1 = 0; sc_s2 = 0; sc_s3 = 0;
        }else{
          ok = true;
          sc_s2 = invK * sqrt(temp3);
          const double temp2 = atan2(2, sc_s2 * sc_Kmax);
          sc_s1 = invK * mod2pi(sc_th0 - temp1 + temp2);
          sc_s3 = invK * mod2pi(sc_thf - temp1 + temp2);
        }

        auto result = std::make_tuple(ok, sc_s1, sc_s2, sc_s3);
        return result;
      }

    static std::tuple<bool,double,double,double> RLR(
      const double sc_th0, const double sc_thf, const double sc_Kmax){
        double sc_s1, sc_s2, sc_s3;
        bool ok = false;

        const double invK = 1 / sc_Kmax;
        const double C = cos(sc_th0) - cos(sc_thf);
        const double S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
        const double temp1 = atan2(C, S);
        const double temp2 = 0.125 * (6 - 4 * pow(sc_Kmax,2) + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));

        if(abs(temp2) > 1){
          ok = false;
          sc_s1 = 0; sc_s2 = 0; sc_s3 = 0;
        }else{
          ok = true;
          sc_s2 = invK * mod2pi(2 * PI - acos(temp2));
          sc_s1 = invK * mod2pi(sc_th0 - temp1 + 0.5 * sc_s2 * sc_Kmax);
          sc_s3 = invK * mod2pi(sc_th0 - sc_thf + sc_Kmax * (sc_s2 - sc_s1));
        }

        auto result = std::make_tuple(ok, sc_s1, sc_s2, sc_s3);
        return result;
      }

    static std::tuple<bool,double,double,double> LRL(
      const double sc_th0, const double sc_thf, const double sc_Kmax){
        double sc_s1, sc_s2, sc_s3;
        bool ok = false;

        const double invK = 1 / sc_Kmax;
        const double C = cos(sc_thf) - cos(sc_th0);
        const double S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
        const double temp1 = atan2(C, S);
        const double temp2 = 0.125 * (6 - 4 * pow(sc_Kmax,2) + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));

        if(abs(temp2) > 1){
          ok = false;
          sc_s1 = 0; sc_s2 = 0; sc_s3 = 0;
        }else{
          ok = true;
          sc_s2 = invK * mod2pi(2 * PI - acos(temp2));
          sc_s1 = invK * mod2pi(temp1 - sc_th0 + 0.5 * sc_s2 * sc_Kmax);
          sc_s3 = invK * mod2pi(sc_thf - sc_th0 + sc_Kmax * (sc_s2 - sc_s1));
        }

        auto result = std::make_tuple(ok, sc_s1, sc_s2, sc_s3);
        return result;
      }

    ////////////////////////////////////////
    ///// Solver

    std::tuple<int, DubinsCurve> dubins_shortest_path(
      const double x0, const double y0, const double th0,
      const double xf, const double yf, const double thf, const double Kmax){

        // log_info("Calculating path...");

        double sc_th0, sc_thf, sc_Kmax, lambda;
        std::tie(sc_th0, sc_thf, sc_Kmax, lambda) = scaleToStandard(x0, y0, th0, xf, yf, thf, Kmax);
        
        int ksigns[6][3] = {{1,0,1},{-1,0,-1},{1,0,-1},{-1,0,1},{-1,1,-1},{1,-1,1}};
        std::vector<std::function<std::tuple<bool,double,double,double>(double,double,double)>> primitives({
          &LSL, &RSR, &LSR, &RSL, &RLR, &LRL
        });

        int pidx = -1;
        double L = INFINITY, sc_s1, sc_s2, sc_s3;

        for(int i=0; i<primitives.size(); i++){
          double sc_s1_c, sc_s2_c, sc_s3_c;
          bool ok;
          std::tie(ok, sc_s1_c, sc_s2_c, sc_s3_c) = primitives[i](sc_th0, sc_thf, sc_Kmax);
          double Lcur = sc_s1_c + sc_s2_c + sc_s3_c;
          if (ok && Lcur<L){
            L = Lcur;
            sc_s1 = sc_s1_c;
            sc_s2 = sc_s2_c;
            sc_s3 = sc_s3_c;
            pidx = i;        
          }
        }
        
        DubinsCurve curve;
        if(pidx>-1){
          double s1, s2, s3;
          std::tie(s1, s2, s3) = scaleFromStandard(lambda, sc_s1, sc_s2, sc_s3);
          curve = getCurve(
            x0, y0, th0,
            s1, s2, s3,
            ksigns[pidx][0]*Kmax, ksigns[pidx][1]*Kmax, ksigns[pidx][2]*Kmax);
          
          // std::ostringstream s;
          // s<<"Feasible curve from x:"<<x0<<", y:"<<y0<<", th:"<<th0<<", to x:"<<xf<<", y:"<<yf<<", th:"<<thf<<" found.";
          // log_info(s.str());
        }else{
          // std::ostringstream s;
          // s<<"Feasible curve from x:"<<x0<<", y:"<<y0<<", th:"<<th0<<", to x:"<<xf<<", y:"<<yf<<", th:"<<thf<<" not found.";
          // log_warn(s.str());
        }
        return std::make_tuple(pidx, curve);
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

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  auto node=std::make_shared<DubinsCalculator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
  return 0;
}