/**
 * @file simple_mujoco_ros2_interface.hpp
 * @brief
 * @author Jeongwoo Hong (jwhong1209@gmail.com)
 * @note
 */

#ifndef SIMPLE_MUJOCO_ROS2_INTERFACE_HPP_
#define SIMPLE_MUJOCO_ROS2_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

/* C++ STL */
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

/* MuJoCo-related */
#include <mujoco/mujoco.h>
#include "simulate.h"

// using namespace rclcpp;
using namespace std::chrono_literals;

namespace mj = ::mujoco;
namespace mju = ::mujoco::util;

struct MujocoCommand
{
  std::vector<double> q_des;
  std::vector<double> dq_des;
  std::vector<double> tau_des;
  rclcpp::Time timestamp;
  bool is_valid = false;

  MujocoCommand() : timestamp(rclcpp::Time(0)), is_valid(false)
  {
  }
};

class SimpleMujocoRos2Interface : public rclcpp::Node
{
private:
  static SimpleMujocoRos2Interface * global_instance_;

  mj::Simulate * sim_;

  // Thread-safe command handling
  MujocoCommand latest_command_;
  std::mutex command_mutex_;
  rclcpp::Time last_command_time_;
  std::atomic<bool> command_received_;

  // Safety parameters
  static constexpr double COMMAND_TIMEOUT_MS = 10.0;  // 10ms timeout
  static constexpr double MAX_COMMAND_AGE_MS = 5.0;   // 5ms max age

  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_cmd_;
  void subJointCommand(const sensor_msgs::msg::JointState::SharedPtr msg);

  static constexpr double state_pub_rate_ = 2ms;
  rclcpp::TimerBase::SharedPtr state_pub_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
  void pubJointState();

  void update();

public:
  SimpleMujocoRos2Interface(mj::Simulate * sim);
  ~SimpleMujocoRos2Interface() = default;

  // Main control interface for mjcb_control callback
  bool getLatestCommand(MujocoCommand & cmd_out);
  void applyJointCommand(const mjModel * m, mjData * d);

  // Safety and status methods
  bool isCommandValid() const;
  bool isCommandTimedOut() const;
  void applyEmergencyStop(const mjModel * m, mjData * d);

  // Utility methods
  // void validateJointLimits(const mjModel * m, MujocoCommand & cmd);

  // Global access for mjcb_control callback
  void setAsGlobalInstance()
  {
    global_instance_ = this;
  }
  static SimpleMujocoRos2Interface * getGlobalInstance()
  {
    return global_instance_;
  }
};

#endif  // SIMPLE_MUJOCO_ROS2_INTERFACE_HPP_