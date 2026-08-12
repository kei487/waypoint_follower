// SPDX-License-Identifier: Apache-2.0

#include "waypoint_manager/waypoint_manager.hpp"

#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <yaml-cpp/yaml.h>

namespace waypoint_manager
{

WaypointManager::WaypointManager(const rclcpp::NodeOptions & options)
: Node("waypoint_manager", options)
{
  getParam();

  initTf();
  initPublisher();
  initSubscription();
  initServiceServer();
//  initActionClient();

  readWaypointYaml();

  initsendGoal();
  initTimer(); 
}

void WaypointManager::getParam()
{
  /*
  this->param_listener_ =
    std::make_shared<waypoint_manager::ParamListener>(this->get_node_parameters_interface());
  this->params_ = param_listener_->get_params();

  waypoint_yaml_path_ = this->params_.waypoint_yaml_path;
  waypoint_radius_ = this->params_.waypoint_radius;
  */
  declare_parameter("waypoint_yaml_path", "waypoint.yaml");
	declare_parameter("waypoint_radius",0.5);

	waypoint_yaml_path_ = get_parameter("waypoint_yaml_path").as_string();
  waypoint_radius_ = get_parameter("waypoint_radius").as_double();
}

void WaypointManager::initTf()
{
  tf_buffer_.reset();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void WaypointManager::initPublisher()
{
  goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "goal_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}


void WaypointManager::initSubscription()
{
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_sub", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
    std::bind(&WaypointManager::cmd_vel_callback, this, std::placeholders::_1));
}

void WaypointManager::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
  geometry_msgs::msg::Twist cmd_vel;
  if(_is_robot_wait){
    cmd_vel.linear.x=0;
    cmd_vel.angular.z=0;
  }else{
    cmd_vel.linear.x=msg->linear.x;
    cmd_vel.angular.z=msg->angular.z;
  }
  vel = cmd_vel.linear.x * cmd_vel.linear.x + cmd_vel.angular.z * cmd_vel.angular.z;

  cmd_vel_pub_->publish(cmd_vel);
}


void WaypointManager::initServiceServer()
{
  auto restart_waypoint_manager =
    [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger_Request> request,
      std::shared_ptr<std_srvs::srv::Trigger_Response> response) -> void {
    (void)request_header;

    _is_robot_wait = false;
    goal_reached_ = false;
    waypoint_id_ = 0;

    if (!waypoints_.waypoints.empty()) {
      sendGoal(waypoints_.waypoints[waypoint_id_].pose);
    }

    response->success = true;
    response->message = "Called /restart_waypoint_manager. Restarted from first waypoint.";
  };
  restart_waypoint_manager_service_server_ =
    create_service<std_srvs::srv::Trigger>("restart_waypoint_manager", restart_waypoint_manager);

}
/*
void WaypointManager::initActionClient()
{
  navigate_to_goal_action_client_ = rclcpp_action::create_client<NavigateToGoal>(
    this->get_node_base_interface(), this->get_node_graph_interface(),
    this->get_node_logging_interface(), this->get_node_waitables_interface(), "navigate_to_goal");
}
*/
void WaypointManager::initTimer()
{
  using namespace std::chrono_literals;

  loop_timer_ = this->create_wall_timer(
    std::chrono::milliseconds{100ms}, std::bind(&WaypointManager::loop, this));
}

void WaypointManager::readWaypointYaml()
{
  YAML::Node waypoints_yaml = YAML::LoadFile(waypoint_yaml_path_);

  waypoints_.header.frame_id = "map";
  waypoints_.header.stamp = rclcpp::Time();
  waypoints_.waypoints.clear();

  if (!waypoints_yaml["waypoints"].IsNull()) {
    for (const auto & waypoint_yaml : waypoints_yaml["waypoints"]) {
      waypoint_manager_msgs::msg::Waypoint waypoint;

      waypoint.id = waypoint_yaml["id"].as<uint32_t>();
      waypoint.pose.position.x = waypoint_yaml["position"]["x"].as<double>();
      waypoint.pose.position.y = waypoint_yaml["position"]["y"].as<double>();
      waypoint.pose.orientation.w = cos(waypoint_yaml["euler_angle"]["z"].as<double>() / 2.);
      waypoint.pose.orientation.z = sin(waypoint_yaml["euler_angle"]["z"].as<double>() / 2.);
      waypoint.robot_wait = waypoint_yaml["robot_wait"]
        ? waypoint_yaml["robot_wait"].as<bool>()
        : false;

      waypoint.function.variable_waypoint_radius.waypoint_radius = static_cast<float>(waypoint_radius_);
      if (waypoint_yaml["functions"].IsDefined()) {
        for (const auto & function : waypoint_yaml["functions"]) {
          if (function["function"].as<std::string>() == "variable_waypoint_radius") {
            if (!function["waypoint_radius"].IsNull()) {
              waypoint.function.variable_waypoint_radius.waypoint_radius =
                function["waypoint_radius"].as<float>();
            }
          }
        }
      }

      waypoints_.waypoints.push_back(waypoint);
    }
  }

  if (waypoints_.waypoints.empty()) {
    RCLCPP_ERROR(
      this->get_logger(), "No waypoints loaded from %s", waypoint_yaml_path_.c_str());
  }
}

void WaypointManager::getMapFrameRobotPose(
  geometry_msgs::msg::PoseStamped & map_frame_robot_pose)
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      "map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.1));
    map_frame_robot_pose.header.frame_id = "map";
    map_frame_robot_pose.header.stamp = transform.header.stamp;
    map_frame_robot_pose.pose.position.x = transform.transform.translation.x;
    map_frame_robot_pose.pose.position.y = transform.transform.translation.y;
    map_frame_robot_pose.pose.position.z = transform.transform.translation.z;
    map_frame_robot_pose.pose.orientation = transform.transform.rotation;
    get_robot_pose_ = true;
  } catch (const tf2::TransformException &) {
    get_robot_pose_ = false;
  }
}

bool WaypointManager::isInsideWaypointArea(
  const geometry_msgs::msg::Pose & robot_pose, const waypoint_manager_msgs::msg::Waypoint & waypoint)
{
  auto distance = std::hypot(
    robot_pose.position.x - waypoint.pose.position.x,
    robot_pose.position.y - waypoint.pose.position.y);

  if (distance < waypoint.function.variable_waypoint_radius.waypoint_radius) {
    return true;
  }

  return false;
}

void WaypointManager::initsendGoal(){
  if (waypoints_.waypoints.empty()) {
    RCLCPP_ERROR(get_logger(), "Cannot send goal: no waypoints loaded");
    return;
  }

  waypoint_id_ = 0;
  goal_reached_ = false;
  RCLCPP_INFO(get_logger(), "Send first goal (waypoint id=%u)", waypoints_.waypoints[waypoint_id_].id);
  sendGoal(waypoints_.waypoints[waypoint_id_].pose);
}

void WaypointManager::sendGoal(const geometry_msgs::msg::Pose & goal)
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

  auto goal_stamp = geometry_msgs::msg::PoseStamped();
  goal_stamp.header.frame_id = "map";
  goal_stamp.header.stamp = rclcpp::Time();
  goal_stamp.pose = goal;

  goal_pub_ -> publish(goal_stamp);
}

/*
void WaypointManager::cancelGoal()
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
void WaypointManager::loop()
{
  if (goal_reached_ || waypoints_.waypoints.empty()) {
    return;
  }

  getMapFrameRobotPose(robot_pose_);
  if (!get_robot_pose_) {
    return;
  }

  const auto & current_waypoint = waypoints_.waypoints[waypoint_id_];

  if (vel < 1e-5 && !_is_robot_wait &&
    !isInsideWaypointArea(robot_pose_.pose, current_waypoint))
  {
    sendGoal(current_waypoint.pose);
    RCLCPP_DEBUG(this->get_logger(), "Re-sent goal for waypoint id=%u", current_waypoint.id);
  }

  if (waypoint_id_ < waypoints_.waypoints.size() - 1) {
    if (isInsideWaypointArea(robot_pose_.pose, current_waypoint)) {
      _is_robot_wait = current_waypoint.robot_wait;
      ++waypoint_id_;
      RCLCPP_INFO(
        get_logger(), "Waypoint id=%u reached. Sending next goal (waypoint id=%u)",
        current_waypoint.id, waypoints_.waypoints[waypoint_id_].id);
      sendGoal(waypoints_.waypoints[waypoint_id_].pose);
    }
  } else if (isInsideWaypointArea(robot_pose_.pose, current_waypoint)) {
    goal_reached_ = true;
    RCLCPP_INFO(get_logger(), "All waypoints reached (final waypoint id=%u)", current_waypoint.id);
  }
}

}  // namespace waypoint_manager

int main(int argc, char **argv)
{
	rclcpp::init(argc,argv);
  rclcpp::NodeOptions opt;
	auto node = std::make_shared<waypoint_manager::WaypointManager>(opt);
	rclcpp::spin(node);
	return 0;
}
