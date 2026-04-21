#include "assignment1/action_client.hpp"

#include <functional>
#include <memory>

#include "rclcpp_components/register_node_macro.hpp"

namespace assignment1
{

ActionClient::ActionClient(const rclcpp::NodeOptions & options)
: Node("action_client", options)
{
  action_client_ = rclcpp_action::create_client<NavigateToPose>( this, "/navigate_to_pose");

  goal_sub_ = this->create_subscription<assignment1::msg::UiGoal>( "/ui_goal", 10,
    std::bind(&ActionClient::goal_callback, this, std::placeholders::_1));

  cancel_sub_ = this->create_subscription<std_msgs::msg::Bool>( "/ui_cancel", 10,
    std::bind(&ActionClient::cancel_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Action Client started");
  RCLCPP_INFO(this->get_logger(), "Listening on /ui_goal and /ui_cancel");
}

void ActionClient::goal_callback(const assignment1::msg::UiGoal::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Received goal from UI: x=%.2f y=%.2f theta=%.2f",
    msg->x, msg->y, msg->theta);

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available");
    return;
  }

  NavigateToPose::Goal goal_msg;
  goal_msg.x = msg->x;
  goal_msg.y = msg->y;
  goal_msg.theta = msg->theta;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;

  options.goal_response_callback =
    [this](const GoalHandleNavigate::SharedPtr & goal_handle)
    {
      this->goal_response_callback(goal_handle);
    };

  options.feedback_callback =
    [this](
      GoalHandleNavigate::SharedPtr goal_handle,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
      this->feedback_callback(goal_handle, feedback);
    };

  options.result_callback =
    [this](const GoalHandleNavigate::WrappedResult & result)
    {
      this->result_callback(result);
    };

  action_client_->async_send_goal(goal_msg, options);
}

void ActionClient::cancel_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) {
    return;
  }

  if (!current_goal_handle_) {
    RCLCPP_WARN(this->get_logger(), "No active goal to delete");
    return;
  }

  RCLCPP_WARN(this->get_logger(), "Request cancel received from UI");

  action_client_->async_cancel_goal(current_goal_handle_);
}

void ActionClient::goal_response_callback(const GoalHandleNavigate::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal rejected by the server");
    return;
  }

  current_goal_handle_ = goal_handle;
  RCLCPP_INFO(this->get_logger(), "Goal accepted by the server");
}

void ActionClient::feedback_callback(
  GoalHandleNavigate::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Feedback -> x=%.2f y=%.2f theta=%.2f dist=%.2f angle=%.2f",
    feedback->current_x,
    feedback->current_y,
    feedback->current_theta,
    feedback->distance_error,
    feedback->angle_error);
}

void ActionClient::result_callback(const GoalHandleNavigate::WrappedResult & result)
{
  current_goal_handle_.reset();

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(
        this->get_logger(),
        "Goal completed: success=%s, message=%s",
        result.result->success ? "true" : "false",
        result.result->message.c_str());
      break;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal aborted by the server");
      break;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Goal cancelled");
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Unkonwn result");
      break;
  }
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(assignment1::ActionClient)