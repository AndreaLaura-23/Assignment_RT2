#ifndef ASSIGNMENT1__ACTION_SERVER_HPP_
#define ASSIGNMENT1__ACTION_SERVER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
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
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigate> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleNavigate> goal_handle);

  void execute(
    const std::shared_ptr<GoalHandleNavigate> goal_handle);
};

}  

#endif