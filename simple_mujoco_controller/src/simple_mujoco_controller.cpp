#include "simple_mujoco_controller.hpp"

#include <iostream>

using namespace std;

SimpleMujocoController::SimpleMujocoController(const double Hz)
  : Node("simple_mujoco_controller"), loop_rate_(Hz)
{
  /* initialize states */
  q_des_ << -0.7854, 1.5708;
  q_mes_ << -0.7854, 1.5708;
  dq_des_.setZero();
  dq_mes_.setZero();
  tau_des_.setZero();
  tau_mes_.setZero();

  p_init_ << 1.4142, 0.0;
  p_des_ << 1.4142, 0.0;
  p_cal_.setZero();
  v_des_.setZero();
  v_cal_.setZero();
  a_des_.setZero();
  a_cal_.setZero();

  /* set control parameters */
  kp_ << 50.0, 50.0;
  kd_ << 5, 5;

  /* create model and planner objects */
  robot_ = std::make_unique<DoublePendulumModel<double>>();
  if (!robot_)
    cout << "Error: Double Pendulum Model is not created!" << endl;
  else
    cout << "Double Pendulum Model is created" << endl;

  planner_ = std::make_unique<TrajectoryGenerator<double>>();
  if (!planner_)
    cout << "Error: Trajectory Generator is not created!" << endl;
  else
    cout << "Trajectory Generator is created" << endl;

  /* Define QoS, Publisher, Subscriber */
  // const auto qos_sim = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
  // sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
  //   "mj_joint_state", qos_sim,
  //   std::bind(&SimpleMujocoController::subMujocoState, this, std::placeholders::_1));

  // const auto qos_ctrl = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
  // pub_joint_cmd_ = this->create_publisher<sensor_msgs::msg::JointState>(
  //   "mj_joint_command", qos_ctrl,
  //   std::bind(&SimpleMujocoController::pubControllerState, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "SimpleMujocoController initialized");
}

void SimpleMujocoController::update()
{
  auto prev_time = this->now();

  while (rclcpp::ok())
  {
    rclcpp::spin_some(this->shared_from_this());

    // if (!bIsSimRunning_)
    // {
    //   RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //                        "Simulation is not running");
    //   continue;
    // }

    auto current_time = this->now();
    loop_time_ = (current_time - prev_time).seconds();
    prev_time = current_time;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Controller loop time: %.3f ms", loop_time_ * 1000.0);

    loop_rate_.sleep();
  }
}