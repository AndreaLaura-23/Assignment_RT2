#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "my_robot_action_cpp/action/move_to_x.hpp"

using namespace std::chrono_literals;

class MoveToXActionClient : public rclcpp::Node
{
public:
  using MoveToX = my_robot_action_cpp::action::MoveToX;
  using GoalHandleMoveToX = rclcpp_action::ClientGoalHandle<MoveToX>;

  MoveToXActionClient(double target_x)
  : Node("move_to_x_client"),
    target_x_(target_x)
  {
    client_ptr_ = rclcpp_action::create_client<MoveToX>(this, "move_to_x");
  }

  void send_goal()
  {
    if (!client_ptr_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = MoveToX::Goal();
    goal_msg.target_x = target_x_;

    RCLCPP_INFO(this->get_logger(), "Sending goal: target_x = %.2f", target_x_);

    auto send_goal_options = rclcpp_action::Client<MoveToX>::SendGoalOptions();

    send_goal_options.feedback_callback =
      std::bind(&MoveToXActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    send_goal_options.result_callback =
      std::bind(&MoveToXActionClient::result_callback, this, std::placeholders::_1);

    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<MoveToX>::SharedPtr client_ptr_;
  double target_x_;

  void feedback_callback(
    GoalHandleMoveToX::SharedPtr,
    const std::shared_ptr<const MoveToX::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Current x: %.2f | Remaining distance: %.2f",
      feedback->current_x,
      feedback->distance_remaining
    );
  }

  void result_callback(const GoalHandleMoveToX::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Result: success=%s, message=%s",
          result.result->success ? "true" : "false",
          result.result->message.c_str());
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "Unknown result code");
        break;
    }

    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc != 2) {
    std::cerr << "Usage: ros2 run my_robot_action_cpp move_to_x_client <target_x>" << std::endl;
    return 1;
  }

  double target_x = std::atof(argv[1]);

  auto node = std::make_shared<MoveToXActionClient>(target_x);
  node->send_goal();
  rclcpp::spin(node);

  return 0;
}