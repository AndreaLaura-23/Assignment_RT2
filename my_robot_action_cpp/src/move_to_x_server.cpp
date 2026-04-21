#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "my_robot_action_cpp/action/move_to_x.hpp"

using namespace std::chrono_literals;

class MoveToXActionServer : public rclcpp::Node
{
public:
  using MoveToX = my_robot_action_cpp::action::MoveToX;
  using GoalHandleMoveToX = rclcpp_action::ServerGoalHandle<MoveToX>;

  MoveToXActionServer()
  : Node("move_to_x_server"),
    current_x_(0.0),
    odom_received_(false)
  {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&MoveToXActionServer::odom_callback, this, std::placeholders::_1)
    );

    action_server_ = rclcpp_action::create_server<MoveToX>(
      this,
      "move_to_x",
      std::bind(&MoveToXActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveToXActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveToXActionServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "MoveToX Action Server started.");
  }

private:
  rclcpp_action::Server<MoveToX>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  double current_x_;
  bool odom_received_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    odom_received_ = true;
  }

  void stop_robot()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToX::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal target_x = %.2f", goal->target_x);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveToX> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    stop_robot();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveToX> goal_handle)
  {
    std::thread{std::bind(&MoveToXActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveToX> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveToX::Feedback>();
    auto result = std::make_shared<MoveToX::Result>();

    rclcpp::Rate loop_rate(10);
    const double tolerance = 0.05;
    const double speed = 0.2;

    while (rclcpp::ok() && !odom_received_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for /odom...");
      loop_rate.sleep();
    }

    if (!rclcpp::ok()) {
      result->success = false;
      result->message = "ROS interrupted before odom was received";
      goal_handle->abort(result);
      return;
    }

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        stop_robot();
        result->success = false;
        result->message = "Goal canceled";
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      double error = goal->target_x - current_x_;

      feedback->current_x = current_x_;
      feedback->distance_remaining = std::fabs(error);
      goal_handle->publish_feedback(feedback);

      // robot arrived 
      if (std::fabs(error) <= tolerance) {
        stop_robot();
        result->success = true;
        result->message = "Target reached";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal reached. Final x = %.2f", current_x_);
        return;
      }

      geometry_msgs::msg::Twist cmd;

      // move forward or backward
      if (error > 0.0) {
        cmd.linear.x = speed;
      } else {
        cmd.linear.x = -speed;
      }

      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);

      loop_rate.sleep();
    }

    stop_robot();
    result->success = false;
    result->message = "ROS interrupted";
    goal_handle->abort(result);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToXActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}