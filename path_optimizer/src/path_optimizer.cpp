#include "path_optimizer/path_optimizer.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <Eigen/Dense>

using namespace Eigen;

namespace path_optimizer
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

PathOptimizer::PathOptimizer()
: Node("path_optimizer"),
  // initialize parameters
  w_origin_(declare_parameter<float>("w_origin", 0.7)),
  w_smooth_(declare_parameter<float>("w_smooth", 0.3)),
  optimize_torelance_(declare_parameter<float>("optimize_torelance", 0.1))
{
  pub_opt_trajectory_ = create_publisher<Trajectory>("output/trajectory", 1);
  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", 1, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", 1, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });
  //subscribe 
  //autoware_auto_perception_msgs/msg/PredictedObjects
  //perception/object_recognition/objects 
  sub_objects_ = create_subscription<PredictedObjects>(
    "input/objects", 1, [this](const PredictedObjects::SharedPtr msg) { objects_ = msg; });

  
  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 30ms, std::bind(&PathOptimizer::onTimer, this));
}

void PathOptimizer::onTimer()
{
  // check data
  if (!subscribeMessageAvailable()) {
    return;
  }

  //copy trajectory
  opt_trajectory_ = *trajectory_;
  path_smoothing(300);
  //if(objects_->objects.size()>0) avoid_objects();
  //path_smoothing(3);
  pub_opt_trajectory_->publish(opt_trajectory_);
}

bool PathOptimizer::subscribeMessageAvailable()
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

void PathOptimizer::path_smoothing(int max_times){
  Trajectory origin_traj=opt_trajectory_;
  Trajectory pre_traj=opt_trajectory_;
  int count=0;
  double change = optimize_torelance_;
  while (change >= optimize_torelance_&&count<max_times) {
    count++;
    change = 0; // 初期化
    pre_traj=opt_trajectory_;
    for (size_t i = 1; i < origin_traj.points.size() -30; ++i) { // 始点と終点は固定
      opt_trajectory_.points.at(i).pose.position.x -= w_origin_ * (opt_trajectory_.points.at(i).pose.position.x- origin_traj.points.at(i).pose.position.x);
      opt_trajectory_.points.at(i).pose.position.y -= w_origin_ * (opt_trajectory_.points.at(i).pose.position.y- origin_traj.points.at(i).pose.position.y);
      opt_trajectory_.points.at(i).pose.position.x -= w_smooth_ * (2 * opt_trajectory_.points.at(i).pose.position.x - opt_trajectory_.points.at(i-1).pose.position.x - opt_trajectory_.points.at(i+1).pose.position.x);
      opt_trajectory_.points.at(i).pose.position.y -= w_smooth_ * (2 * opt_trajectory_.points.at(i).pose.position.y - opt_trajectory_.points.at(i-1).pose.position.y - opt_trajectory_.points.at(i+1).pose.position.y);
      change += hypot(opt_trajectory_.points.at(i).pose.position.x- origin_traj.points.at(i).pose.position.x,opt_trajectory_.points.at(i).pose.position.y- origin_traj.points.at(i).pose.position.y);
    }
  }
  //std::cout<<"smoothing cnt:" << count<<std::endl;
}

void PathOptimizer::avoid_objects(){
  double ob_x,ob_y,ob_theta;
  double traj_theta,traj_dist;
  bool judgeLR=false;
  bool avoid = false;
  last_collision_idx_=0;
  size_t predict_size=std::min(predict_collision_size_,opt_trajectory_.points.size()-1);
  for(size_t i=0;i<objects_->objects.size();i++){
    for(size_t j=0;j<predict_size;j++){
      avoid = false;
      traj_theta=atan2(opt_trajectory_.points.at(j+1).pose.position.y-opt_trajectory_.points.at(j).pose.position.y,opt_trajectory_.points.at(j+1).pose.position.x-opt_trajectory_.points.at(j).pose.position.x);
      for(size_t path_i=0;path_i<objects_->objects.at(i).kinematics.predicted_paths.size();path_i++){
        for(size_t k=0;k<objects_->objects.at(i).kinematics.predicted_paths.at(path_i).path.size();k++){
          ob_x=objects_->objects.at(i).kinematics.predicted_paths.at(path_i).path.at(k).position.x;
          ob_y=objects_->objects.at(i).kinematics.predicted_paths.at(path_i).path.at(k).position.y;
          traj_dist=hypot(ob_y-opt_trajectory_.points.at(j).pose.position.y,ob_x-opt_trajectory_.points.at(j).pose.position.x);
          if(traj_dist<=collisiion_dist_){
            avoid=true;
            last_collision_idx_=j;
            ob_theta=atan2(ob_y-opt_trajectory_.points.at(j).pose.position.y,ob_x-opt_trajectory_.points.at(j).pose.position.x);
            if(!judgeLR){
              if(ob_theta-traj_theta>0){//objectがtrajectoryの左
                offset_y_ = -3.0;
              }else{//objectがtrajectoryの右
                offset_y_ = 3.0;
              }
              judgeLR=true;
            }
          }
        }
      }
      if(avoid||(last_collision_idx_!=0 &&j>last_collision_idx_&& j<last_collision_idx_+10)){
        opt_trajectory_.points.at(j).pose.position.x+=offset_y_*-sin(traj_theta);
        opt_trajectory_.points.at(j).pose.position.y+=offset_y_*cos(traj_theta);
      }
    }
  }
}

}  // namespace path_optimizer

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_optimizer::PathOptimizer>());
  rclcpp::shutdown();
  return 0;
}
