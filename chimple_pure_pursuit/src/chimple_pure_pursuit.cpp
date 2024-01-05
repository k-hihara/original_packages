#include "chimple_pure_pursuit/chimple_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <Eigen/Dense>

using namespace Eigen;

namespace chimple_pure_pursuit
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

ChimplePurePursuit::ChimplePurePursuit()
: Node("chimple_pure_pursuit"),
  // initialize parameters
  wheel_base_(declare_parameter<float>("wheel_base", 2.14)),
  lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 1.0)),
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.0)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<float>("external_target_vel", 0.0)),
  wheel_tred_(declare_parameter<float>("wheel_tred", 1.7)),
  max_acc_(declare_parameter<float>("max_acc", 100/3.6)),
  min_acc_(declare_parameter<float>("min_acc", -100/3.6)),
  max_w_acc_(declare_parameter<float>("max_w_acc", 200/180*M_PI)),
  min_w_acc_(declare_parameter<float>("min_w_acc", -200/180*M_PI))
{
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_pp_path_ = create_publisher<Path>("output/purepursuit_path", 1);

  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", 1, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", 1, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 30ms, std::bind(&ChimplePurePursuit::onTimer, this));
}

AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp)
{
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = 0.0;
  cmd.lateral.stamp = stamp;
  cmd.lateral.steering_tire_angle = 0.0;
  return cmd;
}

void ChimplePurePursuit::onTimer()
{
  // check data
  if (!subscribeMessageAvailable()) {
    return;
  }

  // publish zero command
  cmd_ = zeroAckermannControlCommand(get_clock()->now());

  //load current status
  current_longitudinal_vel_ = odometry_->twist.twist.linear.x;
  size_t closest_traj_point_idx_ =
    findNearestIndex(trajectory_->points, odometry_->pose.pose.position);
  if (
    (closest_traj_point_idx_ == trajectory_->points.size() - 1) ||
    (trajectory_->points.size() <= 5)) {
    cmd_.longitudinal.speed = 0.0;
    cmd_.longitudinal.acceleration = -10.0;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "reached to the goal");
  } else {
    //calc_cmd;
    calc_longi_cmd();
    calc_lat_cmd();
    //std::cout<<"target "<<target_longitudinal_vel_<<" mps,  curvature_rad "<<1/curvature_<<" , curvature"<<curvature_<<std::endl;
    //std::cout<<"curvature_limited_vel "<<curvature_limited_vel()<<std::endl;
  }
  pub_cmd_->publish(cmd_);
  gen_planned_path();
  pub_pp_path_->publish(planned_path_);
}

bool ChimplePurePursuit::subscribeMessageAvailable()
{
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "odometry is not available");
    return false;
  }
  if (!trajectory_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "trajectory is not available");
    return false;
  }
  return true;
}

void ChimplePurePursuit::calc_longi_cmd(){
  // get closest trajectory point from current position
  TrajectoryPoint closest_traj_point = trajectory_->points.at(closest_traj_point_idx_);
  // calc longitudinal speed and acceleration
  target_longitudinal_vel_ =
    use_external_target_vel_ ? external_target_vel_ : closest_traj_point.longitudinal_velocity_mps;
  //calc minimum future trajectory speed
  //double min_future_target_vel=target_longitudinal_vel_;
  double min_future_target_vel=255;
  //TrajectoryPoint search_traj_point;
  max_curvature_=straight_curvature_;
  double curvature_i;
  left_curve_=false;
  right_curve_=false;
  //size_t max_curvature_ind=0;
  for(size_t i=closest_traj_point_idx_+future_curvature_idx_;i<closest_traj_point_idx_+future_curvature_idx_+future_curvature_predict_idx_&&i<trajectory_->points.size()-2;i++){
    curvature_i=calc_curvature(i);
    if(curvature_i>judge_curvature_&&i<s_detect_idx_){
      //std::cout<<" left:"<<i<<","<<curvature_i;
      left_curve_=true;
    }else if(curvature_i<-judge_curvature_&&i<s_detect_idx_){
      //std::cout<<" right:"<<i<<","<<curvature_i;
      right_curve_=true;
    }else{}
    //max_curvature_=std::max(max_curvature_,curvature_i);
    if(max_curvature_<curvature_i){
      max_curvature_=curvature_i;
      //max_curvature_ind=i;
    }
  }
  //std::cout<<max_curvature_ind<<" curvature_rad = "<<1/max_curvature_<<std::endl;
  min_future_target_vel=std::min(min_future_target_vel,curvature_limited_vel(calc_curvature(closest_traj_point_idx_+future_curvature_predict_idx_)));
  if(left_curve_&&right_curve_)min_future_target_vel=std::min(min_future_target_vel,s_vel_mps_);
  //if(left_curve_&&right_curve_)std::cout<<"s curve"<<std::endl;
  target_longitudinal_vel_=min_future_target_vel;
  //limit acc
  double diff_target_current_vel=target_longitudinal_vel_ - current_longitudinal_vel_;
  if(diff_target_current_vel>0 && diff_target_current_vel>max_acc_*dt_){//max_acc
    cmd_.longitudinal.speed = current_longitudinal_vel_+max_acc_*dt_;
    cmd_.longitudinal.acceleration =max_acc_;
    //std::cout<<"maxacc limit"<<std::endl;
  }
  else if(diff_target_current_vel<0&&diff_target_current_vel<min_acc_*dt_){//min_acc
    cmd_.longitudinal.speed = current_longitudinal_vel_+min_acc_*dt_;
    cmd_.longitudinal.acceleration =min_acc_;
    //std::cout<<"minacc limit"<<std::endl;
  }
  else{
    cmd_.longitudinal.speed = target_longitudinal_vel_;
    cmd_.longitudinal.acceleration = speed_proportional_gain_ * (target_longitudinal_vel_ - current_longitudinal_vel_);
  } 
  //goal through
  if(trajectory_->points.size()<140)target_longitudinal_vel_=166/3.6;
  cmd_.longitudinal.speed = target_longitudinal_vel_;
  cmd_.longitudinal.acceleration = speed_proportional_gain_ * (target_longitudinal_vel_ - current_longitudinal_vel_);

}

void ChimplePurePursuit::calc_lat_cmd(){
  // calc lateral control
  //// calc lookahead distance
  lookahead_distance_ = lookahead_gain_ * current_longitudinal_vel_ + lookahead_min_distance_;
  lookahead_distance_=std::min(lookahead_distance_,lookahehad_max_distance_);
  /*if(current_longitudinal_vel_>100/3.6){
    lookahead_distance_=lookahehad_max_distance_;
  }*/
  //// calc center coordinate of rear wheel
  double rear_x = odometry_->pose.pose.position.x -
                  wheel_base_ / 2.0 * std::cos(odometry_->pose.pose.orientation.z);
  double rear_y = odometry_->pose.pose.position.y -
                  wheel_base_ / 2.0 * std::sin(odometry_->pose.pose.orientation.z);
  
  //// search lookahead point
  auto lookahead_point_itr = std::find_if(
    trajectory_->points.begin() + closest_traj_point_idx_, trajectory_->points.end(),
    [&](const TrajectoryPoint & point) {
      return std::hypot(point.pose.position.x - rear_x, point.pose.position.y - rear_y) >=
              lookahead_distance_;
    });
  if (lookahead_point_itr == trajectory_->points.end()) {
    lookahead_point_itr == trajectory_->points.end()-1;
  }
  double lookahead_point_x = lookahead_point_itr->pose.position.x;
  double lookahead_point_y = lookahead_point_itr->pose.position.y;

  // calc steering angle for lateral control
  double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) -
                  tf2::getYaw(odometry_->pose.pose.orientation);

  double dist_to_lookahead_point=std::hypot(lookahead_point_y - rear_y, lookahead_point_x - rear_x);
  double str_gain_tmp=std::max(str_gain_,str_gain_*(std::min(1.0,current_longitudinal_vel_/100)));
  cmd_.lateral.steering_tire_angle =std::atan2(2.0 * wheel_base_ * std::sin(alpha), dist_to_lookahead_point)*str_gain_tmp;
  //cmd_.lateral.steering_tire_angle =std::atan2(2.0 * wheel_tred_ * std::sin(alpha), dist_to_lookahead_point);
  curvature_ = (2*current_longitudinal_vel_*sin(alpha))/dist_to_lookahead_point;
  omega_=2*current_longitudinal_vel_*sin(alpha)/lookahead_distance_;
  //std::cout<<"dist to look ahead point: "<<dist_to_lookahead_point<<" , closest_traj_point_idx_: "<<closest_traj_point_idx_<<std::endl;
  //std::cout<<"closest_traj_point_idx_: "<<closest_traj_point_idx_<<" , lookahead_point_itr: "<<lookahead_point_itr<<std::endl;

}

double ChimplePurePursuit::curvature_limited_vel(double curvature){
  //speed[kmph],curvature[curvature] table
  //const double design_speeds[] = {250,180,150,130, 120, 100, 80, 60, 50, 40, 30, 20, 10, 5};
  const double design_speeds[] = {250, 100, 100, 100, 100, 100, 90, 90, 90, 70, 70, 70, 70, 70};
  //const double design_speeds[] = {250, 100, 100, 100, 100, 100, 90, 70, 70, 70, 70, 70, 70, 70};
  const double design_curvatures[] = {straight_curvature_,1.0/3000,1.0/2000,1.0/1200, 1.0 / 1000, 1.0 / 700, 1.0 / 400, 1.0 / 200, 1.0 / 150, 1.0 / 100, 1.0 / 65, 1.0 / 30, 1.0/20, 1.0/10};
  //find_nearest_curvature
  int closest_index = 0;
  double min_diff = std::abs(abs(curvature) - design_curvatures[0]);
  for (size_t i = 1; i < sizeof(design_speeds)/sizeof(design_speeds[0]); ++i) {
      double diff = std::abs(abs(curvature) - design_curvatures[i]);
      if (diff < min_diff) {
          min_diff = diff;
          closest_index = i;
      }
  }
  //std::cout<<"curvature closest_index: "<<closest_index<<std::endl;
  //kmph2mps
  double design_speed_mps = design_speeds[closest_index] / 3.6;
  //double limited_vel = design_speed_mps / (1 + std::abs(curvature_ * design_speed_mps));
  double limited_vel=design_speed_mps;
  return limited_vel;
}

double ChimplePurePursuit::calc_curvature(size_t idx){
  if(idx<2)idx=2;
  if(idx>trajectory_size_-2)idx=trajectory_size_-2;
  double v1x = trajectory_->points.at(idx).pose.position.x - trajectory_->points.at(idx-diff_calc_curvature_).pose.position.x;
  double v1y = trajectory_->points.at(idx).pose.position.y - trajectory_->points.at(idx-diff_calc_curvature_).pose.position.y;
  double v2x = trajectory_->points.at(idx+diff_calc_curvature_).pose.position.x - trajectory_->points.at(idx).pose.position.x;
  double v2y = trajectory_->points.at(idx+diff_calc_curvature_).pose.position.y - trajectory_->points.at(idx).pose.position.y;
  // Calculate lengths of vectors
  double len1 = std::sqrt(v1x * v1x + v1y * v1y);
  double len2 = std::sqrt(v2x * v2x + v2y * v2y);
  // Cross product of vectors
  double crossProduct = v1x * v2y - v1y * v2x;
  // Calculate curvature
  double curvature = 2.0 * crossProduct / (len1 * len2 * (len1 + len2));
  return curvature;
}

void ChimplePurePursuit::gen_planned_path(){
  planned_path_.header.stamp=get_clock()->now();
  planned_path_.header.frame_id="base_link";
  planned_path_.poses.clear();
  PoseStamped p;
  double dt=0.1;
  double dt_dist=target_longitudinal_vel_*dt;
  double dt_heading=omega_*dt;
  double px = 0;
  double py = 0;
  double ptheta=0;
  for(int i=0;i<100;i++){
    px+=dt_dist*cos(ptheta);
    py+=dt_dist*sin(ptheta);
    ptheta+=dt_heading;
    p.header=planned_path_.header;
    p.pose.position.x=px;
    p.pose.position.y=py;
    p.pose.position.z=0;
    planned_path_.poses.push_back(p);
  }
}

}  // namespace chimple_pure_pursuit

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<chimple_pure_pursuit::ChimplePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
