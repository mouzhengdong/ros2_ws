#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "cpp_action_example/action/count_until.hpp"

class CountUntilActionClient : public rclcpp::Node {
public:
  using CountUntil = cpp_action_example::action::CountUntil;
  using GoalHandle = rclcpp_action::ClientGoalHandle<CountUntil>;

  CountUntilActionClient() : Node("count_until_client") {
    client_ptr_ = rclcpp_action::create_client<CountUntil>(this, "count_until");

    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = CountUntil::Goal();
    goal_msg.target_number = 10;
    goal_msg.period = 0.5;  // 每 0.5 秒增加一次

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto options = rclcpp_action::Client<CountUntil>::SendGoalOptions();
    options.feedback_callback =
      std::bind(&CountUntilActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    options.result_callback =
      std::bind(&CountUntilActionClient::result_callback, this, std::placeholders::_1);

    client_ptr_->async_send_goal(goal_msg, options);
  }

private:
  rclcpp_action::Client<CountUntil>::SharedPtr client_ptr_;

  void feedback_callback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const CountUntil::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Feedback: current number = %d", feedback->current_number);
  }

  void result_callback(const GoalHandle::WrappedResult & result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Result: reached number = %d", result.result->reached_number);
    } else {
      RCLCPP_WARN(this->get_logger(), "Action failed or was cancelled");
    }

    rclcpp::shutdown();
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CountUntilActionClient>());
  return 0;
}