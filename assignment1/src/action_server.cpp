#include "assignment1/action_server.hpp"

#include <functional>
#include <memory>
#include <thread>

#include "rclcpp_components/register_node_macro.hpp"

namespace assignment1
{

ActionServer::ActionServer(const rclcpp::NodeOptions & options)
: Node("action_server", options)
{
  action_server_ = rclcpp_action::create_server<NavigateToPose>(
    this,
    "/navigate_to_pose",
    std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "Action Server started");
}

rclcpp_action::GoalResponse ActionServer::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Goal received: x=%.2f y=%.2f theta=%.2f",
    goal->x, goal->y, goal->theta);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Request cancel received");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  std::thread{std::bind(&ActionServer::execute, this, goal_handle)}.detach();
}

void ActionServer::execute(
  const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Esecution of the goal started");

  auto feedback = std::make_shared<NavigateToPose::Feedback>();
  auto result = std::make_shared<NavigateToPose::Result>();

  rclcpp::Rate loop_rate(1.0);

  for (int i = 0; i < 5; i++) {
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Goal cancelled";

      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal cancelled");
      return;
    }

    feedback->current_x = static_cast<double>(i);
    feedback->current_y = static_cast<double>(i);
    feedback->current_theta = 0.0;
    feedback->distance_error = 5.0 - static_cast<double>(i);
    feedback->angle_error = 0.0;

    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(
      this->get_logger(),
      "Feedback sent: x=%.2f y=%.2f dist=%.2f",
      feedback->current_x,
      feedback->current_y,
      feedback->distance_error);

    loop_rate.sleep();
  }

  result->success = true;
  result->message = "Goal reached";

  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Goal completed with success");
}

}  

RCLCPP_COMPONENTS_REGISTER_NODE(assignment1::ActionServer)