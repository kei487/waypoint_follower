// SPDX-License-Identifier: Apache-2.0

#include "ike_waypoint_follower/ike_waypoint_follower.hpp"

#include <nav2_util/robot_utils.hpp>

#include <yaml-cpp/yaml.h>

namespace ike_nav
{

IkeWaypointFollower::IkeWaypointFollower(const rclcpp::NodeOptions & options)
: Node("ike_waypoint_follower", options)
{
  getParam();

  initTf();
  initPublisher();
  initSubscription();
//  initServiceServer();
//  initActionClient();
  initTimer();

//  readWaypointYaml();
}

void IkeWaypointFollower::getParam()
{
  this->param_listener_ =
    std::make_shared<ike_waypoint_follower::ParamListener>(this->get_node_parameters_interface());
  this->params_ = param_listener_->get_params();

  waypoint_yaml_path_ = this->params_.waypoint_yaml_path;
  waypoint_radius_ = this->params_.waypoint_radius;
}

void IkeWaypointFollower::initTf()
{
  tf_buffer_.reset();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void IkeWaypointFollower::initPublisher()
{
  waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "goal_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}
/*
void IkeWaypointFollower::initSubscription()
{
  auto waypoints_callback = [&](const ike_nav_msgs::msg::Waypoints::ConstSharedPtr msg) {
    if (!msg->waypoints.size()) {
      this->loop_timer_->cancel();
      this->cancelGoal();
      this->waypoint_id_ = 0;
    }
    if (waypoint_id_ >= msg->waypoints.size()) {
      if (msg->waypoints.size()) {
        waypoint_id_ = msg->waypoints.size() - 1;
      } else {
        waypoint_id_ = 0;
      }
    }
    waypoints_ = *msg;
  };

  waypoints_sub_ = this->create_subscription<ike_nav_msgs::msg::Waypoints>(
    "waypoints", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), waypoints_callback);
}
*/
/*
void IkeWaypointFollower::initServiceServer()
{
  auto load_waypoint_yaml =
    [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      std::shared_ptr<ike_nav_msgs::srv::LoadWaypointYaml::Request> request,
      std::shared_ptr<ike_nav_msgs::srv::LoadWaypointYaml::Response> response) -> void {
    (void)request_header;

    this->loop_timer_->reset();
    waypoint_yaml_path_ = request->waypoint_yaml_path;
    this->readWaypointYaml();
    this->loop_timer_->cancel();
    this->cancelGoal();
    this->waypoint_id_ = 0;

    response->success = true;
  };
  load_waypoint_yaml_service_server_ =
    create_service<ike_nav_msgs::srv::LoadWaypointYaml>("load_waypoint_yaml", load_waypoint_yaml);

  auto start_waypoint_follower =
    [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger_Request> request,
      std::shared_ptr<std_srvs::srv::Trigger_Response> response) -> void {
    (void)request_header;

    this->loop_timer_->reset();
    this->sendGoal(this->waypoints_.waypoints[this->waypoint_id_].pose);

    response->success = true;
    response->message = "Called /start_waypoint_follower. Send goal done.";
  };
  start_waypoint_follower_service_server_ =
    create_service<std_srvs::srv::Trigger>("start_waypoint_follower", start_waypoint_follower);

  auto stop_waypoint_follower =
    [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger_Request> request,
      std::shared_ptr<std_srvs::srv::Trigger_Response> response) -> void {
    (void)request_header;

    this->loop_timer_->cancel();
    this->cancelGoal();

    response->success = true;
    response->message = "Called /stop_waypoint_follower. Cancel goal done.";
  };
  stop_waypoint_follower_service_server_ =
    create_service<std_srvs::srv::Trigger>("stop_waypoint_follower", stop_waypoint_follower);

  auto cancel_waypoint_follower =
    [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger_Request> request,
      std::shared_ptr<std_srvs::srv::Trigger_Response> response) -> void {
    (void)request_header;

    this->loop_timer_->cancel();
    this->cancelGoal();
    this->waypoint_id_ = 0;

    response->success = true;
    response->message = "Called /cancel_waypoint_follower. Cancel goal done.";
  };
  cancel_waypoint_follower_service_server_ =
    create_service<std_srvs::srv::Trigger>("cancel_waypoint_follower", cancel_waypoint_follower);
}

void IkeWaypointFollower::initActionClient()
{
  navigate_to_goal_action_client_ = rclcpp_action::create_client<NavigateToGoal>(
    this->get_node_base_interface(), this->get_node_graph_interface(),
    this->get_node_logging_interface(), this->get_node_waitables_interface(), "navigate_to_goal");
}
*/
void IkeWaypointFollower::initTimer()
{
  using namespace std::chrono_literals;

  loop_timer_ = this->create_wall_timer(
    std::chrono::milliseconds{100ms}, std::bind(&IkeWaypointFollower::loop, this));
}
/*
void IkeWaypointFollower::readWaypointYaml()
{
  YAML::Node waypoints_yaml = YAML::LoadFile(waypoint_yaml_path_);

  waypoints_.header.frame_id = "map";
  waypoints_.header.stamp = rclcpp::Time();
  waypoints_.waypoints.clear();

  if (!waypoints_yaml["waypoints"].IsNull()) {
    for (const auto & waypoint_yaml : waypoints_yaml["waypoints"]) {
      ike_nav_msgs::msg::Waypoint waypoint;

      waypoint.id = waypoint_yaml["id"].as<uint32_t>();
      waypoint.pose.position.x = waypoint_yaml["position"]["x"].as<double>();
      waypoint.pose.position.y = waypoint_yaml["position"]["y"].as<double>();
      waypoint.pose.orientation.w = cos(waypoint_yaml["euler_angle"]["z"].as<double>() / 2.);
      waypoint.pose.orientation.z = sin(waypoint_yaml["euler_angle"]["z"].as<double>() / 2.);

      if (waypoint_yaml["functions"].IsDefined()) {
        for (const auto & function : waypoint_yaml["functions"]) {
          if (function["function"].as<std::string>() == "variable_waypoint_radius") {
            if (!function["waypoint_radius"].IsNull()) {
              waypoint.function.variable_waypoint_radius.waypoint_radius =
                function["waypoint_radius"].as<float>();
            }
          }
        }
      } else {
        waypoint.function.variable_waypoint_radius.waypoint_radius = waypoint_radius_;
      }

      waypoints_.waypoints.push_back(waypoint);
    }
  }

  waypoints_pub_->publish(waypoints_);
}
*/
void IkeWaypointFollower::getMapFrameRobotPose(
  geometry_msgs::msg::PoseStamped & map_frame_robot_pose)
{
  geometry_msgs::msg::PoseStamped pose;
  if (nav2_util::getCurrentPose(pose, *tf_buffer_)) {
    map_frame_robot_pose = pose;
    get_robot_pose_ = true;
  }
}

bool IkeWaypointFollower::isInsideWaypointArea(
  const geometry_msgs::msg::Pose & robot_pose, const ike_nav_msgs::msg::Waypoint & waypoint)
{
  auto distance = std::hypot(
    robot_pose.position.x - waypoint.pose.position.x,
    robot_pose.position.y - waypoint.pose.position.y);

  if (distance < waypoint.function.variable_waypoint_radius.waypoint_radius) {
    return true;
  }

  return false;
}

void IkeWaypointFollower::sendGoal(const geometry_msgs::msg::Pose & goal)
{
  /*
  using namespace std::chrono_literals;

  if (!navigate_to_goal_action_client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    return;
  }

  auto goal_msg = NavigateToGoal::Goal();
  auto goal_stamp = geometry_msgs::msg::PoseStamped();
  goal_stamp.header.frame_id = "map";
  goal_stamp.header.stamp = rclcpp::Time();
  goal_stamp.pose = goal;
  goal_msg.pose = goal_stamp;
  goal_msg.waypoint_id = waypoint_id_ + 1;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<NavigateToGoal>::SendGoalOptions();
  auto goal_handle_future =
    navigate_to_goal_action_client_->async_send_goal(goal_msg, send_goal_options);
  */

  pub_goal_ -> publish(goal);
  RCLCPP_INFO(this->get_logger(), "Sent goal");
}

/*
void IkeWaypointFollower::cancelGoal()
{
  using namespace std::chrono_literals;

  if (!navigate_to_goal_action_client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    return;
  }

  auto goal_msg = NavigateToGoal::Goal();

  RCLCPP_INFO(this->get_logger(), "Cancel goal");

  auto goal_handle_future = navigate_to_goal_action_client_->async_cancel_all_goals();
}
*/
void IkeWaypointFollower::loop()
{
  RCLCPP_INFO(get_logger(), "Run IkeWaypointFollower::loop");
  /*
  if (!waypoints_.waypoints.size()) {
    this->loop_timer_->cancel();
    //this->cancelGoal();
    this->waypoint_id_ = 0;
  }
  */

  getMapFrameRobotPose(robot_pose_);
  if (get_robot_pose_) {
    if (waypoints_.waypoints.size() - 1 != waypoint_id_) {
      if (isInsideWaypointArea(robot_pose_.pose, waypoints_.waypoints[waypoint_id_])) {
        RCLCPP_INFO(get_logger(), "Send next goal");
        sendGoal(waypoints_.waypoints[++waypoint_id_].pose);
      }
    } else {
      if (isInsideWaypointArea(robot_pose_.pose, waypoints_.waypoints[waypoint_id_])) {
        RCLCPP_INFO(get_logger(), "Goal Reached");
      }
    }
  }
}

}  // namespace ike_nav


