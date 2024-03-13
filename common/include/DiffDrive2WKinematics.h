#ifndef diffdrivekinematics_h_
#define diffdrivekinematics_h_

#include "Kinematics.h"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <nav_msgs/msg/odometry.hpp>

class DiffDrive2WKinematics : public Kinematics
{
public:
	DiffDrive2WKinematics();

	void execForwKin(const sensor_msgs::msg::JointState::SharedPtr js, nav_msgs::msg::Odometry& odom) override;
	void execInvKin(const geometry_msgs::msg::Twist::SharedPtr twist, trajectory_msgs::msg::JointTrajectory& traj) override;

	void setAxisLength(double dLength);
	void setWheelDiameter(double dDiam);

private:
	double m_phiAbs = 0;
	nav_msgs::msg::Odometry m_curr_odom;

	double m_dAxisLength = 0;
	double m_dDiam = 0;

};

#endif
