#ifndef PATH_OPTIMIZER_HPP_
#define PATH_OPTIMIZER_HPP_

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace path_optimizer {

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjectKinematics;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedPath;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::Path;

class PathOptimizer : public rclcpp::Node {
 public:
  explicit PathOptimizer();
  
  // subscribers
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  //subscribe /perception/object_recognition/objects
  rclcpp::Subscription<PredictedObjects>::SharedPtr sub_objects_;
  
  // publishers
  rclcpp::Publisher<Trajectory>::SharedPtr pub_opt_trajectory_;
  
  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // updated by subscribers
  Trajectory::SharedPtr trajectory_;
  Trajectory opt_trajectory_;
  Odometry::SharedPtr odometry_;
  PredictedObjects::SharedPtr objects_;
  
  // path optimize parameters
  //w_origin+w_smooth=1, 0<w_smooth_<0.5, w_smooth: strength of smoothing
  const double w_origin_;
  const double w_smooth_;
  const double optimize_torelance_;
  
 private:
  void onTimer();
  bool subscribeMessageAvailable();
  void path_smoothing(int max_times);
  void avoid_objects();
  size_t predict_collision_size_=50;
  size_t last_collision_idx_;
  double collisiion_dist_ = 3.0;
  double offset_y_;//collision avoid offset
};

}  // namespace path_optimizer

#endif  // PATH_OPTIMIZER_