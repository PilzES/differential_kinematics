#ifndef kinematics_h_
#define kinematics_h_

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "rclcpp/rclcpp.hpp"

class Kinematics
{
public:
	virtual void execForwKin(const sensor_msgs::msg::JointState::SharedPtr js, nav_msgs::msg::Odometry& odom) = 0;
	virtual void execInvKin(const geometry_msgs::msg::Twist::SharedPtr Twist, trajectory_msgs::msg::JointTrajectory& Jt) = 0;
};
#endif
