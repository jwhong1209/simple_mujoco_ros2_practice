/**
 * @file simple_mujoco_ros2_interface.hpp
 * @brief
 * @author Jeongwoo Hong (jwhong1209@gmail.com)
 * @note
 */

#ifndef SIMPLE_MUJOCO_ROS2_INTERFACE_HPP_
#define SIMPLE_MUJOCO_ROS2_INTERFACE_HPP_

/* ROS2 Packages */
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

/* C++ STL */
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

/* MuJoCo-related */
#include "array_safety.h"
#include "simulate.h"

using namespace std::chrono_literals;
namespace mj = ::mujoco;

class SimpleMujocoRos2Interface : public rclcpp::Node
{
public:
  struct MujocoCommand
  {
    int mode = 2;  // 0: position, 1: velocity, 2: torque
    std::vector<double> q_des;
    std::vector<double> dq_des;
    std::vector<double> tau_des;
  };
  std::shared_ptr<MujocoCommand> mj_cmd_ptr_;

  SimpleMujocoRos2Interface(mj::Simulate * sim, const double Hz);
  ~SimpleMujocoRos2Interface() = default;

  void physicsThread(const char * filename);
  void physicsLoop();
  void spinnerLoop();

  // static SimpleMujocoRos2Interface & getInstance()
  // {
  //   static SimpleMujocoRos2Interface instance;
  //   return instance;
  // }

private:
  const int kDoF = 2;

  mj::Simulate * sim_;
  mjModel * m_ = nullptr;
  mjData * d_ = nullptr;
  mjModel * loadModel(const char * filename);

  rclcpp::Rate loop_rate_;

  //* ----- Publisher / Subscriber -----------------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_cmd_;
  void subMujocoCommand(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_gui_cmd_;
  void subGuiCommand(const std_msgs::msg::Bool::SharedPtr msg);
  bool bIsSimRunTriggered_ = true;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
  std::vector<double> q_mes_, dq_mes_, tau_mes_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_sim_run_;
  bool bIsSimReady_ = false;

  void pubMujocoState();

  //* ----- METHOD _--------------------------------------------------------------------------------
  void write();  // copy member variable to mujoco data
  void read();   // copy mujoco data to member variable
  static void controlCallback(const mjModel * m, mjData * d);
  
  // Static instance pointer for callback access
  static SimpleMujocoRos2Interface* instance_;
  void printCameraView()
  {
    std::cout << "azimuth: " << sim_->cam.azimuth << " distance: " << sim_->cam.distance
              << " elevation: " << sim_->cam.elevation << " lookat: " << sim_->cam.lookat[0] << ", "
              << sim_->cam.lookat[1] << ", " << sim_->cam.lookat[2] << std::endl;
  }
};

#endif  // SIMPLE_MUJOCO_ROS2_INTERFACE_HPP_