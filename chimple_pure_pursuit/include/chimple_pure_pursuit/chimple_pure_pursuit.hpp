#ifndef CHIMPLE_PURE_PURSUIT_HPP_
#define CHIMPLE_PURE_PURSUIT_HPP_

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace chimple_pure_pursuit {

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::Path;

class ChimplePurePursuit : public rclcpp::Node {
 public:
  explicit ChimplePurePursuit();
  
  // subscribers
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  
  // publishers
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<Path>::SharedPtr pub_pp_path_;
  
  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // updated by subscribers
  Trajectory::SharedPtr trajectory_;
  Odometry::SharedPtr odometry_;
  
  // pure pursuit parameters
  const double wheel_base_;
  const double lookahead_gain_;
  const double lookahead_min_distance_;
  const double speed_proportional_gain_;
  const bool use_external_target_vel_;
  const double external_target_vel_;
  const double wheel_tred_;
  const double max_acc_;
  const double min_acc_;
  const double max_w_acc_;
  const double min_w_acc_;

 private:
  void onTimer();
  bool subscribeMessageAvailable();
  void calc_longi_cmd();
  void calc_lat_cmd();
  double vel_curvature_check();
  double current_longitudinal_vel_;
  double target_longitudinal_vel_;
  double lookahead_distance_;
  double lookahehad_max_distance_=80;
  double curvature_;
  double omega_;
  double curvature_limited_vel(double curvature);
  double calc_curvature(size_t idx);
  AckermannControlCommand cmd_;
  size_t closest_traj_point_idx_;
  size_t future_curvature_idx_=2; 
  size_t future_curvature_predict_idx_=100;
  size_t s_detect_idx_=60;
  size_t trajectory_size_=150;

  Path planned_path_;
  void gen_planned_path();
  double dt_=0.03;
  double target_curvature_;
  double straight_curvature_ = 0.00001;
  double judge_curvature_=1.0/200;
  double s_vel_mps_=40/3.6;
  bool left_curve_ =false;
  bool right_curve_ =false;
  double max_curvature_;
  double str_gain_=2.0;//1.8
  size_t diff_calc_curvature_=2;
};

}  // namespace chimple_pure_pursuit

double kmph2mps(double vel_kmph){ return vel_kmph/3.6;}
double mps2kmph(double vel_mps){ return vel_mps*3.6;}
double deg2rad(double deg){return deg/180*M_PI;}
double rad2deg(double rad){return rad/M_PI*180;}

#endif  // CHIMPLE_PURE_PURSUIT_HPP_