#include "rclcpp/rclcpp.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using std::placeholders::_1;

class DriveBridge : public rclcpp::Node
{
public:
    DriveBridge()
        : Node("drive_bridge_node")
    {
      drive_pub = this->create_publisher<sensor_msgs::msg::JointState>("drives/joint_states", 1000);
      drive_sub = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("drives/joint_trajectory", 10, std::bind(&DriveBridge::receiveJointSpeed, this, _1));
      timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&DriveBridge::publishJointState, this));
    }

private:
    void receiveJointSpeed(const trajectory_msgs::msg::JointTrajectory::SharedPtr trajectory_msg)
    {
      traj_ = trajectory_msg;
    }

    void publishJointState()
    {
      if (traj_)
      {
          joint_state_msg.header.stamp = this->get_clock()->now();
          joint_state_msg.name = traj_->joint_names;
          joint_state_msg.velocity = traj_->points[0].velocities;
          drive_pub->publish(joint_state_msg);
      }
    }

    sensor_msgs::msg::JointState joint_state_msg;
    trajectory_msgs::msg::JointTrajectory::SharedPtr traj_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr drive_pub;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr drive_sub;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveBridge>());
    rclcpp::shutdown();
    return 0;
}