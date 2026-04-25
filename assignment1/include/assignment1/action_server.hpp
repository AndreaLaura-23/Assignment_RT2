#ifndef ASSIGNMENT1__ACTION_SERVER_HPP_
#define ASSIGNMENT1__ACTION_SERVER_HPP_

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.hpp"

#include "assignment1/action/navigate_to_pose.hpp"

namespace assignment1
{

class ActionServer : public rclcpp::Node
{
public:
  using NavigateToPose = assignment1::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  explicit ActionServer(const rclcpp::NodeOptions & options);

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigate> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleNavigate> goal_handle);

  void stop_robot();

  void execute(
    const std::shared_ptr<GoalHandleNavigate> goal_handle);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;

  std::string moved_frame_;
  std::string world_frame_;
  std::string goal_frame_;
};

}

#endif  // ASSIGNMENT1__ACTION_SERVER_HPP_