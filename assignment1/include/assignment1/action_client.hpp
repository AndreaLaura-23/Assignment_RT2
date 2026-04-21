#ifndef ASSIGNMENT1__ACTION_CLIENT_HPP_
#define ASSIGNMENT1__ACTION_CLIENT_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "assignment1/action/navigate_to_pose.hpp"
#include "assignment1/msg/ui_goal.hpp"
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
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

  rclcpp::Subscription<assignment1::msg::UiGoal>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_sub_;

  GoalHandleNavigate::SharedPtr current_goal_handle_;

  void goal_callback(const assignment1::msg::UiGoal::SharedPtr msg);
  void cancel_callback(const std_msgs::msg::Bool::SharedPtr msg);

  void goal_response_callback(const GoalHandleNavigate::SharedPtr & goal_handle);

  void feedback_callback(
    GoalHandleNavigate::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);

  void result_callback(const GoalHandleNavigate::WrappedResult & result);
};

}  

#endif  