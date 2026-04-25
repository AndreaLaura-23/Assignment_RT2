#include "assignment1/action_server.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/exceptions.hpp"
#include "tf2/utils.h"

namespace assignment1
{

using std::placeholders::_1;
using std::placeholders::_2;

ActionServer::ActionServer(const rclcpp::NodeOptions & options)
: Node("action_server", options)
{
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    10);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom",
    10,
    std::bind(&ActionServer::odom_callback, this, _1));

  goal_frame_ = this->declare_parameter<std::string>(
    "target_frame_name",
    "goal_frame");

  moved_frame_ = this->declare_parameter<std::string>(
    "moved_frame_name",
    "base_link");

  world_frame_ = this->declare_parameter<std::string>(
    "world_frame_name",
    "odom");

  tf_broadcaster_ =
    std::make_shared<tf2_ros::TransformBroadcaster>(this);

  static_broadcaster_ =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());

  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  geometry_msgs::msg::TransformStamped static_tf;
  static_tf.header.stamp = this->get_clock()->now();
  static_tf.header.frame_id = "base_footprint";
  static_tf.child_frame_id = "base_link";
  static_tf.transform.translation.x = 0.0;
  static_tf.transform.translation.y = 0.0;
  static_tf.transform.translation.z = 0.0;
  static_tf.transform.rotation.x = 0.0;
  static_tf.transform.rotation.y = 0.0;
  static_tf.transform.rotation.z = 0.0;
  static_tf.transform.rotation.w = 1.0;

  static_broadcaster_->sendTransform(static_tf);

  geometry_msgs::msg::TransformStamped t_init;
  t_init.header.stamp = this->get_clock()->now();
  t_init.header.frame_id = world_frame_;
  t_init.child_frame_id = moved_frame_;
  t_init.transform.translation.x = 0.0;
  t_init.transform.translation.y = 0.0;
  t_init.transform.translation.z = 0.0;
  t_init.transform.rotation.x = 0.0;
  t_init.transform.rotation.y = 0.0;
  t_init.transform.rotation.z = 0.0;
  t_init.transform.rotation.w = 1.0;

  tf_broadcaster_->sendTransform(t_init);

  action_server_ = rclcpp_action::create_server<NavigateToPose>(
    this,
    "/navigate_to_pose",
    std::bind(&ActionServer::handle_goal, this, _1, _2),
    std::bind(&ActionServer::handle_cancel, this, _1),
    std::bind(&ActionServer::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "Action Server started");
}

void ActionServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = world_frame_;
  t.child_frame_id = moved_frame_;

  t.transform.translation.x = msg->pose.pose.position.x;
  t.transform.translation.y = msg->pose.pose.position.y;
  t.transform.translation.z = msg->pose.pose.position.z;

  t.transform.rotation = msg->pose.pose.orientation;

  tf_broadcaster_->sendTransform(t);
}

rclcpp_action::GoalResponse ActionServer::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Goal received: x=%.2f y=%.2f theta=%.2f",
    goal->x,
    goal->y,
    goal->theta);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  (void)goal_handle;

  RCLCPP_WARN(this->get_logger(), "Cancel request received");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  std::thread{
    std::bind(&ActionServer::execute, this, goal_handle)
  }.detach();
}

void ActionServer::stop_robot()
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 0.0;
  velocity.angular.z = 0.0;

  cmd_vel_pub_->publish(velocity);
}

void ActionServer::execute(
  const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Execution of the goal started");

  auto feedback = std::make_shared<NavigateToPose::Feedback>();
  auto result = std::make_shared<NavigateToPose::Result>();

  rclcpp::Rate rate(10);

  const double distance_tolerance = 0.05;
  const double angle_tolerance = 0.05;

  const double scale_forward_speed = 0.5;
  const double scale_rotation_rate = 1.0;

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      stop_robot();

      result->success = false;
      result->message = "Goal cancelled";

      goal_handle->canceled(result);

      RCLCPP_WARN(this->get_logger(), "Goal cancelled");

      return;
    }

    geometry_msgs::msg::TransformStamped t;

    try {
      t = tf_buffer_->lookupTransform(
        moved_frame_,
        goal_frame_,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not transform %s to %s: %s",
        goal_frame_.c_str(),
        moved_frame_.c_str(),
        ex.what());

      rate.sleep();
      continue;
    }

    const double x_error = t.transform.translation.x;
    const double y_error = t.transform.translation.y;

    const double distance_error =
      std::sqrt(std::pow(x_error, 2) + std::pow(y_error, 2));

    const double angle_error = std::atan2(y_error, x_error);

    geometry_msgs::msg::Twist velocity;

    if (distance_error > distance_tolerance) {
      velocity.linear.x = scale_forward_speed * distance_error;
      velocity.angular.z = scale_rotation_rate * angle_error;

      cmd_vel_pub_->publish(velocity);

      feedback->current_x = x_error;
      feedback->current_y = y_error;
      feedback->current_theta = 0.0;
      feedback->distance_error = distance_error;
      feedback->angle_error = angle_error;

      goal_handle->publish_feedback(feedback);

      RCLCPP_INFO(
        this->get_logger(),
        "Moving: x_error=%.2f y_error=%.2f distance=%.2f angle=%.2f",
        x_error,
        y_error,
        distance_error,
        angle_error);

      rate.sleep();
      continue;
    }

    const double qz = t.transform.rotation.z;
    const double qw = t.transform.rotation.w;
    const double final_angle_error = 2.0 * std::atan2(qz, qw);

    if (std::fabs(final_angle_error) > angle_tolerance) {
      velocity.linear.x = 0.0;
      velocity.angular.z = scale_rotation_rate * final_angle_error;

      cmd_vel_pub_->publish(velocity);

      feedback->current_x = x_error;
      feedback->current_y = y_error;
      feedback->current_theta = final_angle_error;
      feedback->distance_error = distance_error;
      feedback->angle_error = final_angle_error;

      goal_handle->publish_feedback(feedback);

      RCLCPP_INFO(
        this->get_logger(),
        "Final alignment: angle_error=%.2f",
        final_angle_error);

      rate.sleep();
      continue;
    }

    stop_robot();

    result->success = true;
    result->message = "Goal reached";

    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Goal reached");

    return;
  }

  stop_robot();

  result->success = false;
  result->message = "ROS stopped";

  goal_handle->abort(result);
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(assignment1::ActionServer)