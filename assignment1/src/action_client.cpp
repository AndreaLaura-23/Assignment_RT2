#include "assignment1/action_client.hpp"

#include <functional>
#include <memory>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace assignment1
{

using namespace std::placeholders;

ActionClient::ActionClient(const rclcpp::NodeOptions & options) : Node("action_client", options), cancel_sent_(false)
{
  action_client_ = rclcpp_action::create_client<NavigateToPose>( this, "/navigate_to_pose");

  goal_sub_ = this->create_subscription<assignment1::msg::UiGoal>( "/ui_goal", 10, std::bind(&ActionClient::goal_callback, this, _1));
  
  cancel_sub_ = this->create_subscription<std_msgs::msg::Bool>( "/ui_cancel", 10, std::bind(&ActionClient::cancel_callback, this, _1));

  status_pub_ = this->create_publisher<std_msgs::msg::String>( "/goal_status", 10);

  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  goal_frame_name_ = this->declare_parameter<std::string>( "target_frame_name", "goal_frame");

  world_frame_name_ = this->declare_parameter<std::string>( "world_frame_name", "odom");

  RCLCPP_INFO(this->get_logger(), "Action Client started");
  RCLCPP_INFO(this->get_logger(), "Listening on /ui_goal and /ui_cancel");
}

void ActionClient::goal_callback(const assignment1::msg::UiGoal::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Received goal from UI: x=%.2f y=%.2f theta=%.2f",
    msg->x, msg->y, msg->theta);

  send_goal(msg->x, msg->y, msg->theta);
}

  void ActionClient::send_goal(double x_goal, double y_goal, double theta_goal)
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available");
    return;
  }


  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = world_frame_name_;
  t.child_frame_id = goal_frame_name_;

  t.transform.translation.x = x_goal;
  t.transform.translation.y = y_goal;
  t.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta_goal);

  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  static_broadcaster_->sendTransform(t);

  cancel_sent_ = false;

  NavigateToPose::Goal goal_msg;
  goal_msg.x = x_goal;
  goal_msg.y = y_goal;
  goal_msg.theta = theta_goal;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;

  options.goal_response_callback =
    std::bind(&ActionClient::goal_response_callback, this, _1);

  options.feedback_callback =
    std::bind(&ActionClient::feedback_callback, this, _1, _2);

  options.result_callback =
    std::bind(&ActionClient::result_callback, this, _1);

  action_client_->async_send_goal(goal_msg, options);
}

void ActionClient::cancel_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) {
    return;
  }

  if (cancel_sent_) {
    RCLCPP_WARN(this->get_logger(), "Cancel request already sent");
    return;
  }

  RCLCPP_WARN(this->get_logger(), "Cancel request received from UI");

  action_client_->async_cancel_all_goals();

  cancel_sent_ = true;
}

void ActionClient::goal_response_callback( const GoalHandleNavigate::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal rejected by the server");
    current_goal_handle_.reset();
    return;
  }

  current_goal_handle_ = goal_handle;

  RCLCPP_INFO(this->get_logger(), "Goal accepted by the server");
}

void ActionClient::feedback_callback( GoalHandleNavigate::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
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

void ActionClient::result_callback( const GoalHandleNavigate::WrappedResult & result)
{
  current_goal_handle_.reset();

  std_msgs::msg::String status_msg;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(
        this->get_logger(),
        "Goal completed: success=%s, message=%s",
        result.result->success ? "true" : "false",
        result.result->message.c_str());

      status_msg.data = "SUCCEEDED";
      status_pub_->publish(status_msg);
      break;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal aborted by the server");

      status_msg.data = "ABORTED";
      status_pub_->publish(status_msg);
      break;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Goal cancelled");

      status_msg.data = "CANCELLED";
      status_pub_->publish(status_msg);
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result");

      status_msg.data = "UNKNOWN";
      status_pub_->publish(status_msg);
      break;
  }
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(assignment1::ActionClient)