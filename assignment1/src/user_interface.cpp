#include <atomic>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <unistd.h>
#include <sys/select.h>

#include "rclcpp/rclcpp.hpp"

#include "assignment1/msg/ui_goal.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

std::atomic<bool> cancel_requested{false};
std::atomic<bool> goal_running{false};

std::string goal_state = "STILL";

int pipe_fd[2];

class UserInterface : public rclcpp::Node
{
public:
  UserInterface()
  : Node("user_interface")
  {
    goal_pub_ = this->create_publisher<assignment1::msg::UiGoal>(
      "/ui_goal",
      10);

    cancel_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/ui_cancel",
      10);

    status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/goal_status",
      10,
      std::bind(&UserInterface::status_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "User interface node started");
  }

  void input_loop()
  {
    if (pipe(pipe_fd) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create pipe");
      return;
    }

    while (rclcpp::ok()) {
      cancel_requested = false;
      goal_running = false;
      goal_state = "STILL";

      double x;
      double y;
      double theta;

      std::cout << "\n=== New Goal ===\n";

      std::cout << "Insert x target: ";
      check_input(x);

      std::cout << "Insert y target: ";
      check_input(y);

      std::cout << "Insert theta target: ";
      check_input(theta);

      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

      assignment1::msg::UiGoal goal_msg;
      goal_msg.x = x;
      goal_msg.y = y;
      goal_msg.theta = theta;

      goal_pub_->publish(goal_msg);
      goal_running = true;

      RCLCPP_INFO(
        this->get_logger(),
        "Goal sent: x=%.2f y=%.2f theta=%.2f",
        x,
        y,
        theta);

      std::thread cancel_thread(&UserInterface::check_cancel_request, this);

      rclcpp::Rate rate(20);

      while (rclcpp::ok() && goal_running) {
        rclcpp::spin_some(this->get_node_base_interface());

        if (cancel_requested) {
          std_msgs::msg::Bool cancel_msg;
          cancel_msg.data = true;

          cancel_pub_->publish(cancel_msg);

          std::cout << "\nGOAL CANCEL REQUEST SENT\n";

          goal_running = false;
          break;
        }

        if (goal_state == "SUCCEEDED") {
          std::cout << "\nGOAL REACHED\n";
          goal_running = false;
          break;
        }

        if (goal_state == "CANCELLED") {
          std::cout << "\nGOAL CANCELLED\n";
          goal_running = false;
          break;
        }

        if (goal_state == "ABORTED") {
          std::cout << "\nGOAL FAILED\n";
          goal_running = false;
          break;
        }

        rate.sleep();
      }

      char signal_byte = 1;
      write(pipe_fd[1], &signal_byte, 1);

      if (cancel_thread.joinable()) {
        cancel_thread.join();
      }

      std::cout << "\nPress ENTER for a new goal or 'q' + ENTER to quit...\n";

      std::string end_cmd;
      std::getline(std::cin, end_cmd);

      if (end_cmd == "q" || end_cmd == "Q") {
        std::cout << "\nExiting...\n";
        break;
      }
    }

    close(pipe_fd[0]);
    close(pipe_fd[1]);
  }

private:
  rclcpp::Publisher<assignment1::msg::UiGoal>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cancel_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;

  void status_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    goal_state = msg->data;
  }

  void check_input(double & value)
  {
    while (!(std::cin >> value)) {
      std::cout << "Invalid input. Insert a number: ";
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
  }

  void check_cancel_request()
  {
    std::cout << "\nPress 'c' + ENTER to cancel the goal or wait...\n";

    while (rclcpp::ok() && goal_running) {
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(STDIN_FILENO, &fds);
      FD_SET(pipe_fd[0], &fds);

      int max_fd = std::max(STDIN_FILENO, pipe_fd[0]) + 1;

      int ret = select(max_fd, &fds, nullptr, nullptr, nullptr);

      if (ret < 0) {
        return;
      }

      if (FD_ISSET(pipe_fd[0], &fds)) {
        char buf;
        read(pipe_fd[0], &buf, 1);
        return;
      }

      if (FD_ISSET(STDIN_FILENO, &fds)) {
        std::string cmd;
        std::getline(std::cin, cmd);

        if (cmd == "c" || cmd == "C") {
          cancel_requested = true;
          return;
        }
      }
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<UserInterface>();

  node->input_loop();

  rclcpp::shutdown();

  return 0;
}