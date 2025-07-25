#ifndef SIMPLE_MUJOCO_CONTROLLER_HPP_
#define SIMPLE_MUJOCO_CONTROLLER_HPP_

/* C++ STL */
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

/* ROS2 Packages */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

/* Custom Libraries*/
#include "DoublePendulumModel.hpp"
#include "TrajectoryGenerator.hpp"
#include "eigen_types.hpp"

enum class TrajectoryType
{
  TELEOP = 0,
  CIRCLE,
};

class SimpleMujocoController : public rclcpp::Node
{
public:
  SimpleMujocoController(const double Hz);
  ~SimpleMujocoController() = default;

  void update();

private:
  std::unique_ptr<DoublePendulumModel<double>> robot_;
  std::unique_ptr<TrajectoryGenerator<double>> planner_;

  int iter_ = 0;
  double loop_time_ = 0.0;

  bool b_traj_start_ = false;
  double traj_time_ = 0.0;

  TrajectoryType traj_type_ = TrajectoryType::CIRCLE;

  const int kDoF = 2;

  Vec2<double> q_des_, q_mes_;      // desired / measured joint position
  Vec2<double> dq_des_, dq_mes_;    // desired / measured joint velocity
  Vec2<double> tau_des_, tau_mes_;  // desired / measured joint torque

  Vec2<double> p_init_;
  Vec2<double> p_des_, p_cal_;  // desired / calculated EE Cartesian position
  Vec2<double> v_des_, v_cal_;  // desired / calculated EE Cartesian velocity
  Vec2<double> a_des_, a_cal_;  // desired / calculated EE Cartesian acceleration

  Vec2<double> kp_;  // proportional gain
  Vec2<double> kd_;  // derviative gain

  //* ----- ROS Interface --------------------------------------------------------------------------
  rclcpp::Rate loop_rate_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
  void subMujocoState(const sensor_msgs::msg::JointState::SharedPtr msg);
  bool bIsSimRunning_ = false;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_cmd_;
  void pubControllerState();
};

#endif  // SIMPLE_MUJOCO_CONTROLLER_HPP_