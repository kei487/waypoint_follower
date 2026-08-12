// SPDX-License-Identifier: Apache-2.0

#include "waypoint_manager/waypoint_manager.hpp"

#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

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

  readWaypointCsv();

  initsendGoal();
  initTimer(); 
}

void WaypointManager::getParam()
{
  /*
  this->param_listener_ =
    std::make_shared<waypoint_manager::ParamListener>(this->get_node_parameters_interface());
  this->params_ = param_listener_->get_params();

  waypoint_csv_path_ = this->params_.waypoint_csv_path;
  waypoint_radius_ = this->params_.waypoint_radius;
  */
  declare_parameter("waypoint_csv_path", "waypoint.csv");
  declare_parameter("waypoint_radius", 0.5);

  waypoint_csv_path_ = get_parameter("waypoint_csv_path").as_string();
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

void WaypointManager::readWaypointCsv()
{
  auto trim = [](const std::string & text) {
    const auto first =
      std::find_if_not(text.begin(), text.end(), [](unsigned char c) { return std::isspace(c); });
    if (first == text.end()) {
      return std::string{};
    }
    const auto last = std::find_if_not(
      text.rbegin(), text.rend(), [](unsigned char c) { return std::isspace(c); }).base();
    return std::string(first, last);
  };

  auto to_lower = [](std::string text) {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return text;
  };

  auto split_csv_row = [](const std::string & line) {
    std::vector<std::string> columns;
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ',')) {
      columns.emplace_back(token);
    }
    return columns;
  };

  auto parse_double = [&trim](const std::string & text, double & value) {
    try {
      size_t processed = 0U;
      const auto trimmed = trim(text);
      if (trimmed.empty()) {
        return false;
      }
      value = std::stod(trimmed, &processed);
      return processed == trimmed.size();
    } catch (const std::exception &) {
      return false;
    }
  };

  auto parse_bool = [&trim, &to_lower](const std::string & text, bool & value) {
    const auto trimmed = to_lower(trim(text));
    if (trimmed.empty()) {
      return false;
    }
    if (trimmed == "1" || trimmed == "true" || trimmed == "yes") {
      value = true;
      return true;
    }
    if (trimmed == "0" || trimmed == "false" || trimmed == "no") {
      value = false;
      return true;
    }
    return false;
  };

  auto header_has_extended_columns = [&trim, &to_lower](const std::vector<std::string> & header) {
    for (const auto & column : header) {
      const auto name = to_lower(trim(column));
      if (name == "waypoint_radius" || name == "robot_wait") {
        return true;
      }
    }
    return false;
  };

  std::ifstream ifs(waypoint_csv_path_);
  if (!ifs.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open waypoint CSV: %s", waypoint_csv_path_.c_str());
    return;
  }

  waypoints_.header.frame_id = "map";
  waypoints_.header.stamp = rclcpp::Time();
  waypoints_.waypoints.clear();

  std::string line;
  if (!std::getline(ifs, line)) {
    RCLCPP_ERROR(this->get_logger(), "Empty waypoint CSV: %s", waypoint_csv_path_.c_str());
    return;
  }

  const auto header = split_csv_row(line);
  const bool extended = header_has_extended_columns(header);
  const std::size_t min_columns = extended ? 10U : 8U;

  uint32_t fallback_id = 0;
  while (std::getline(ifs, line)) {
    if (trim(line).empty()) {
      continue;
    }

    const auto columns = split_csv_row(line);
    if (columns.size() < min_columns) {
      RCLCPP_WARN(
        this->get_logger(), "Skipping malformed CSV row (columns=%zu): %s",
        columns.size(), line.c_str());
      continue;
    }

    waypoint_manager_msgs::msg::Waypoint waypoint;

    double id_value = 0.0;
    if (parse_double(columns[0], id_value)) {
      waypoint.id = static_cast<uint32_t>(id_value);
    } else {
      waypoint.id = fallback_id;
    }

    if (!parse_double(columns[1], waypoint.pose.position.x) ||
      !parse_double(columns[2], waypoint.pose.position.y) ||
      !parse_double(columns[3], waypoint.pose.position.z) ||
      !parse_double(columns[4], waypoint.pose.orientation.x) ||
      !parse_double(columns[5], waypoint.pose.orientation.y) ||
      !parse_double(columns[6], waypoint.pose.orientation.z) ||
      !parse_double(columns[7], waypoint.pose.orientation.w))
    {
      RCLCPP_WARN(this->get_logger(), "Skipping CSV row with invalid pose: %s", line.c_str());
      continue;
    }

    waypoint.robot_wait = false;
    waypoint.function.variable_waypoint_radius.waypoint_radius =
      static_cast<float>(waypoint_radius_);

    if (extended) {
      double radius = waypoint_radius_;
      if (parse_double(columns[8], radius)) {
        waypoint.function.variable_waypoint_radius.waypoint_radius = static_cast<float>(radius);
      }

      bool wait = false;
      if (parse_bool(columns[9], wait)) {
        waypoint.robot_wait = wait;
      }
    }

    waypoints_.waypoints.push_back(waypoint);
    ++fallback_id;
  }

  if (waypoints_.waypoints.empty()) {
    RCLCPP_ERROR(
      this->get_logger(), "No waypoints loaded from %s", waypoint_csv_path_.c_str());
  } else {
    RCLCPP_INFO(
      this->get_logger(), "Loaded %zu waypoints from %s",
      waypoints_.waypoints.size(), waypoint_csv_path_.c_str());
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
