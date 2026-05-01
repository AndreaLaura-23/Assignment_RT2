#ifndef ASSIGNMENT1__ACTION_CLIENT_HPP_
#define ASSIGNMENT1__ACTION_CLIENT_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2_ros/static_transform_broadcaster.h"

#include "assignment1/action/navigate_to_pose.hpp"
#include "assignment1/msg/ui_goal.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

namespace assignment1
{

class ActionClient : public rclcpp::Node
{
public:
  using NavigateToPose = assignment1::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit ActionClient(const rclcpp::NodeOptions & options);

private:
  void goal_callback(const assignment1::msg::UiGoal::SharedPtr msg);

  void cancel_callback(const std_msgs::msg::Bool::SharedPtr msg);

  void send_goal(double x_goal, double y_goal, double theta_goal);

  void goal_response_callback(const GoalHandleNavigate::SharedPtr & goal_handle);

  void feedback_callback(GoalHandleNavigate::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);

  void result_callback(const GoalHandleNavigate::WrappedResult & result);

  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

  rclcpp::Subscription<assignment1::msg::UiGoal>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  GoalHandleNavigate::SharedPtr current_goal_handle_;

  std::string world_frame_name_;
  std::string goal_frame_name_;

  bool cancel_sent_;
};


}  

#endif  