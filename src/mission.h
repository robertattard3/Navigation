#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/point.hpp>


using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class Mission : public rclcpp::Node
{
public:
    Mission();

private:
    // Add a new goal (position)
    void sendGoal(const geometry_msgs::msg::Point & msg);
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goalPoint;

};

#endif // MISSION_H