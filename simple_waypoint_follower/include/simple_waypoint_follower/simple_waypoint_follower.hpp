// SPDX-License-Identifier: Apache-2.0

#ifndef SIMPLE_WAYPOINT_FOLLOWER__SIMPLE_WAYPOINT_FOLLOWER_HPP_
#define SIMPLE_WAYPOINT_FOLLOWER__SIMPLE_WAYPOINT_FOLLOWER_HPP_

// #include "simple_waypoint_follower_parameter/simple_waypoint_follower_parameter.hpp"

#include <rclcpp/rclcpp.hpp>
//#include <rclcpp_action/rclcpp_action.hpp>

//#include "simple_waypoint_follower_msgs/action/navigate_to_goal.hpp"
#include "simple_waypoint_follower_msgs/msg/waypoints.hpp"
// #include "simple_waypoint_follower_msgs/srv/load_waypoint_yaml.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
//#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

//using NavigateToGoal = simple_waypoint_follower_msgs::action::NavigateToGoal;
//using GoalHandleNavigateToGoal = rclcpp_action::ServerGoalHandle<NavigateToGoal>;

namespace simple_waypoint_follower
{
class SimpleWaypointFollower : public rclcpp::Node
{
public:
  explicit SimpleWaypointFollower(const rclcpp::NodeOptions & options);

protected:
  void getParam();

  void initTf();
  void initPublisher();
  void initSubscription();
//  void initServiceServer();
//  void initActionClient();
  void initTimer();

  void readWaypointYaml();
  void getMapFrameRobotPose(geometry_msgs::msg::PoseStamped & map_frame_robot_pose);
  bool isInsideWaypointArea(
    const geometry_msgs::msg::Pose & robot_pose, const simple_waypoint_follower_msgs::msg::Waypoint & waypoint);
  void sendGoal(const geometry_msgs::msg::Pose & goal);
//  void cancelGoal();

  void loop();

private:
  // clang-format off
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
//  rclcpp::Publisher<simple_waypoint_follower_msgs::msg::Waypoints>::SharedPtr waypoints_pub_;
//  rclcpp::Subscription<simple_waypoint_follower_msgs::msg::Waypoints>::SharedPtr waypoints_sub_;
//  rclcpp::Service<simple_waypoint_follower_msgs::srv::LoadWaypointYaml>::SharedPtr load_waypoint_yaml_service_server_;
//  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_waypoint_follower_service_server_;
//  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_waypoint_follower_service_server_;
//  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_waypoint_follower_service_server_;
//  rclcpp_action::Client<NavigateToGoal>::SharedPtr navigate_to_goal_action_client_;
  // clang-format on

  rclcpp::TimerBase::SharedPtr loop_timer_;

//  std::shared_ptr<simple_waypoint_follower::ParamListener> param_listener_;
//  simple_waypoint_follower::Params params_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string waypoint_yaml_path_;
  simple_waypoint_follower_msgs::msg::Waypoints waypoints_;
  double waypoint_radius_;

  u_int32_t waypoint_id_;
  geometry_msgs::msg::PoseStamped robot_pose_;

  bool get_robot_pose_;
};

}  // namespace simple_waypoint_follower

#endif  // SIMPLE_WAYPOINT_FOLLOWER__SIMPLE_WAYPOINT_FOLLOWER_HPP_
