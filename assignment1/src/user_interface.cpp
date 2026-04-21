#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "assignment1/msg/ui_goal.hpp"

class UserInterface : public rclcpp::Node
{
public:
  UserInterface() : Node("user_interface")
  {
    goal_pub_ = this->create_publisher<assignment1::msg::UiGoal>("/ui_goal", 10);
    cancel_pub_ = this->create_publisher<std_msgs::msg::Bool>("/ui_cancel", 10);

    RCLCPP_INFO(this->get_logger(), "User interface node started");
  }

  void input_loop()
  {
    while (rclcpp::ok()) {
      std::cout << "\n[g] send goal   [c] cancel   [q] quit\n> ";
      char cmd;
      std::cin >> cmd;

      if (cmd == 'g') {
        double x, y, theta;
        std::cout << "Insert x y theta: ";
        std::cin >> x >> y >> theta;

        assignment1::msg::UiGoal msg;
        msg.x = x;
        msg.y = y;
        msg.theta = theta;

        goal_pub_->publish(msg);
        RCLCPP_INFO(
          this->get_logger(),
          "Published goal: x=%.2f y=%.2f theta=%.2f",
          x, y, theta);
      }
      else if (cmd == 'c') {
        std_msgs::msg::Bool msg;
        msg.data = true;
        cancel_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published cancel request");
      }
      else if (cmd == 'q') {
        RCLCPP_INFO(this->get_logger(), "Closing user interface");
        break;
      }
      else {
        std::cout << "Unknown command. Use g, c, or q.\n";
      }
    }
  }

private:
  rclcpp::Publisher<assignment1::msg::UiGoal>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cancel_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UserInterface>();
  node->input_loop();
  rclcpp::shutdown();
  return 0;
}