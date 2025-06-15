#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "cpp_action_example/action/count_until.hpp"

class CountUntilActionServer : public rclcpp::Node {
public:
  using CountUntil = cpp_action_example::action::CountUntil;
  using GoalHandleCountUntil = rclcpp_action::ServerGoalHandle<CountUntil>;

  CountUntilActionServer() : Node("count_until_action_server") {
    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<CountUntil>(
      this,
      "count_until",
      std::bind(&CountUntilActionServer::handle_goal, this, _1, _2),
      std::bind(&CountUntilActionServer::handle_cancel, this, _1),
      std::bind(&CountUntilActionServer::handle_accepted, this, _1)
    );
  }

private:
  rclcpp_action::Server<CountUntil>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CountUntil::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal: count until %d with %.2fs interval", goal->target_number, goal->period);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleCountUntil> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCountUntil> goal_handle)
  {
    std::thread{std::bind(&CountUntilActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleCountUntil> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CountUntil::Feedback>();
    auto result = std::make_shared<CountUntil::Result>();

    rclcpp::Rate rate(1.0 / goal->period);  // 控制频率
    int count = 0;

    while (rclcpp::ok() && count < goal->target_number)
    {
      if (goal_handle->is_canceling()) {
        result->reached_number = count;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled at %d", count);
        return;
      }

      feedback->current_number = count;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Feedback: %d", count);

      count++;
      rate.sleep();
    }

    result->reached_number = count;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded with %d", count);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CountUntilActionServer>());
  rclcpp::shutdown();
  return 0;
}