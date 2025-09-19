#include "mission.h"

Mission::Mission() : Node("navigation")
{

    client_ = rclcpp_action::create_client<NavigateToPose>(this,"navigate_to_pose");
    // subscribe to if user inputs manual goals
    goalPoint = this->create_subscription<geometry_msgs::msg::Point>("/userGoal", 10,
      std::bind(&Mission::sendGoal, this, std::placeholders::_1));

}


void Mission::sendGoal(const geometry_msgs::msg::Point & msg)
{
  if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(get_logger(), "navigate_to_pose action server not available");
    return;
  }

  // Build PoseStamped (Nav2 is 2D; keep z constant)
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = "map";
  ps.header.stamp = this->now();
  ps.pose.position.x = msg.x;
  ps.pose.position.y = msg.y;

  NavigateToPose::Goal goal;
  goal.pose = ps;

  RCLCPP_INFO(get_logger(), "Navigating to goal: x=%.2f, y=%.2f", msg.x, msg.y);

  auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  opts.goal_response_callback = [this](GoalHandleNavigateToPose::SharedPtr handle){
    if (!handle) {
      RCLCPP_WARN(this->get_logger(), "Goal rejected by server.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted.");
    }
  };
  opts.result_callback =
    [this](const GoalHandleNavigateToPose::WrappedResult & result) {
      RCLCPP_INFO(this->get_logger(), "Navigation finished with result code: %d",
                  static_cast<int>(result.code));
      
    };

  client_->async_send_goal(goal, opts);
}

