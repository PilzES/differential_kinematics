#include "../../common/include/Kinematics.h"
#include "../../common/include/DiffDrive2WKinematics.h"
#include "rclcpp/rclcpp.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;

class PlatformCtrlNode: public rclcpp::Node 
{
public:
	PlatformCtrlNode(): Node("differential_kinematics_node") {}

	int init() {
		odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1000);
		drive_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("drives/joint_trajectory", 1000);
		cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&PlatformCtrlNode::receiveCmd, this, _1));
		drive_sub = this->create_subscription<sensor_msgs::msg::JointState>("drives/joint_states", 10, std::bind(&PlatformCtrlNode::receiveOdo, this, _1));
		odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
		
		this->declare_parameter<double>("wheelDiameter", 0.125);
		this->declare_parameter<double>("robotWidth", 0.33);
		this->declare_parameter<std::string>("odomFrame", "odom");
		this->declare_parameter<std::string>("robotBaseFrame", "base_footprint");

		this->get_parameter("wheelDiameter", wheelDiameter);
		this->get_parameter("robotWidth", axisLength);
		this->get_parameter("odomFrame", odomFrame);
		this->get_parameter("robotBaseFrame", robotBaseFrame);
	
		DiffDrive2WKinematics* diffKin = new DiffDrive2WKinematics();
		diffKin->setWheelDiameter(wheelDiameter);
		diffKin->setAxisLength(axisLength);
		kin = diffKin;
		return 0;
	}

	void receiveCmd(const geometry_msgs::msg::Twist::SharedPtr twist) {
		trajectory_msgs::msg::JointTrajectory traj;
		kin->execInvKin(twist, traj);
		drive_pub->publish(traj);
	}

	void receiveOdo(const sensor_msgs::msg::JointState::SharedPtr js) {
		nav_msgs::msg::Odometry odom;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_footprint";
		kin->execForwKin(js, odom);
		odom_pub->publish(odom);

		//odometry transform:
		geometry_msgs::msg::TransformStamped odom_trans;
		odom_trans.header.stamp = odom.header.stamp;
		odom_trans.header.frame_id = odomFrame;
		odom_trans.child_frame_id = robotBaseFrame;
		odom_trans.transform.translation.x = odom.pose.pose.position.x;
		odom_trans.transform.translation.y = odom.pose.pose.position.y;
		odom_trans.transform.translation.z = odom.pose.pose.position.z;
		odom_trans.transform.rotation = odom.pose.pose.orientation;
		odom_broadcaster->sendTransform(odom_trans);
	}

private:
	std::mutex m_node_mutex;
	Kinematics* kin = 0;
	
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
	rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr drive_pub;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr drive_sub;
	std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
	
	double wheelDiameter = 0.0;
	double axisLength = 0.0;
	std::string odomFrame;
	std::string robotBaseFrame;
};

int main (int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto nh = std::make_shared<PlatformCtrlNode>();
	if(nh->init() != 0) {
			RCLCPP_ERROR_STREAM(nh->get_logger(),"differential_kinematics_node: init failed!");
		}
	rclcpp::spin(nh);
	
	return 0;
}
