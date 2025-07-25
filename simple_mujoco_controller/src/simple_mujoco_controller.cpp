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
  try
  {
    robot_ = std::make_unique<DoublePendulumModel<double>>();
    RCLCPP_INFO(this->get_logger(), "Double Pendulum Model is created");

    planner_ = std::make_unique<TrajectoryGenerator<double>>();
    RCLCPP_INFO(this->get_logger(), "Trajectory Generator is created");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to create robot model or planner: %s", e.what());
    throw;
  }

  /* Define Subscriber */
  const auto qos_sim_state = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  sub_sim_state_ = this->create_subscription<std_msgs::msg::Bool>(
    "mj_sim_state", qos_sim_state,
    std::bind(&SimpleMujocoController::subSimulationState, this, std::placeholders::_1));

  const auto qos_jnt_state = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
  sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "mj_joint_state", qos_jnt_state,
    std::bind(&SimpleMujocoController::subJointState, this, std::placeholders::_1));

  /* Define Publisher */
  const auto qos_jnt_command = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
  pub_joint_cmd_ =
    this->create_publisher<sensor_msgs::msg::JointState>("mj_joint_command", qos_jnt_command);

  RCLCPP_INFO(this->get_logger(), "SimpleMujocoController initialized");
}

void SimpleMujocoController::update()
{
  rclcpp::Time prev_time = this->now();
  bool bWasSimRunning = false;

  while (rclcpp::ok())
  {
    rclcpp::spin_some(this->shared_from_this());

    /* wait until simulation is running */
    if (!bIsSimRunning_)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Simulation is not running");
      bWasSimRunning = false;
      prev_time = this->now();
      continue;
    }

    /* calculate loop time */
    rclcpp::Time current_time = this->now();
    if (!bWasSimRunning)
    {
      prev_time = current_time;
      bWasSimRunning = true;
      continue;
    }
    dt_ = (current_time - prev_time).seconds();
    loop_time_ += dt_;
    prev_time = current_time;
    // RCLCPP_INFO(this->get_logger(), "time: %.3f sec, timestep: %.3f ms", loop_time_, dt_ * 1000);

    //* ----- Kinematics / Dynamics ----------------------------------------------------------------
    /* update joint states */
    robot_->setJointStates(q_mes_, dq_mes_);

    /* compute kinematics */
    p_cal_ = robot_->position();

    Mat2<double> J = robot_->jacobian();
    Mat2<double> J_t = J.transpose();  // Jacobian transpose
    Mat2<double> J_inv = J.inverse();
    Mat2<double> J_t_inv = J_t.inverse();
    v_cal_ = robot_->velocity();

    /* compute dynamics */
    Mat2<double> M = robot_->inertia();
    Vec2<double> tau_c = robot_->coriolis();
    Vec2<double> tau_g = robot_->gravity();

    //* ----- Set Desired Trajectory ---------------------------------------------------------------
    switch (traj_type_)
    {
    case TrajectoryType::CIRCLE: {
      const bool bIsTrajRunning = (traj_start_time_ <= loop_time_ && loop_time_ < traj_end_time_);

      if (bIsTrajRunning)
      {
        if (!bTrajStarted_)
        {
          p_init_ = p_cal_;
          bTrajStarted_ = true;
        }
        else
        {
          const int repeat(3);
          double radius(0.1);
          double freq = repeat / (traj_end_time_ - traj_start_time_);

          p_des_ = planner_->circular(loop_time_, traj_start_time_, radius, freq, p_init_);
        }
      }
      else
      {
        p_des_ = p_cal_;
        v_des_.setZero();
        a_des_.setZero();
        bTrajStarted_ = false;
      }
      break;
    }

    default:
      p_des_ = p_cal_;
      v_des_.setZero();
      a_des_.setZero();
      break;
    }

    //* ----- Control Law --------------------------------------------------------------------------
    Vec2<double> Fc = kp_.cwiseProduct(p_des_ - p_cal_) + kd_.cwiseProduct(v_des_ - v_cal_);
    tau_des_ = J_t * Fc;

    /* joint-space control */
    tau_des_ += tau_g;  // qfrc_bias = tau_g
    this->pubJointCommand();

    loop_rate_.sleep();
  }
}

void SimpleMujocoController::subSimulationState(const std_msgs::msg::Bool::SharedPtr msg)
{
  bIsSimRunning_ = msg->data;
}

void SimpleMujocoController::subJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->name.size() != kDoF || msg->position.size() != kDoF || msg->velocity.size() != kDoF ||
      msg->effort.size() != kDoF)
  {
    RCLCPP_ERROR(this->get_logger(), "Joint state size is not correct");
    return;
  }

  for (int i = 0; i < kDoF; ++i)
  {
    q_mes_(i) = msg->position[i];
    dq_mes_(i) = msg->velocity[i];
    tau_mes_(i) = msg->effort[i];
  }
}

void SimpleMujocoController::pubJointCommand()
{
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = this->now();
  msg.name.resize(kDoF);
  msg.position.resize(kDoF);
  msg.velocity.resize(kDoF);
  msg.effort.resize(kDoF);

  msg.name = { "joint1", "joint2" };
  for (int i = 0; i < kDoF; ++i)
  {
    msg.position[i] = q_des_(i);
    msg.velocity[i] = dq_des_(i);
    msg.effort[i] = tau_des_(i);
  }
  pub_joint_cmd_->publish(msg);
}
