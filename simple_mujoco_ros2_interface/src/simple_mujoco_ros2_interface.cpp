#include "simple_mujoco_ros2_interface.hpp"

#include <rclcpp/qos.hpp>

SimpleMujocoRos2Interface * SimpleMujocoRos2Interface::global_instance_ = nullptr;

SimpleMujocoRos2Interface::SimpleMujocoRos2Interface(mj::Simulate * sim)
  : Node("simple_mujoco_ros2_interface"), sim_(sim), command_received_(false)
{
  // Initialize timestamps
  last_command_time_ = this->now();

  // QoS settings for real-time performance
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
               .reliability(rclcpp::ReliabilityPolicy::BestEffort)
               .durability(rclcpp::DurabilityPolicy::Volatile);

  // Subscription for joint commands from controller (1kHz)
  sub_joint_cmd_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_command", qos,
    std::bind(&SimpleMujocoRos2Interface::subJointCommand, this, std::placeholders::_1));

  // Publisher for joint states to other nodes
  pub_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);

  // Timer for publishing joint states at 500Hz
  state_pub_timer_ = this->create_wall_timer(
    state_pub_rate_, std::bind(&SimpleMujocoRos2Interface::pubJointState, this));

  RCLCPP_INFO(this->get_logger(), "SimpleMujocoRos2Interface initialized");
}

void SimpleMujocoRos2Interface::subJointCommand(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(command_mutex_);

  // Validate message
  if (msg->position.empty() && msg->velocity.empty() && msg->effort.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Received empty joint command");
    return;
  }

  // Update latest command
  latest_command_.q_des = msg->position;
  latest_command_.dq_des = msg->velocity;
  latest_command_.tau_des = msg->effort;
  latest_command_.timestamp =
    msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ? this->now() : msg->header.stamp;
  latest_command_.is_valid = true;

  last_command_time_ = this->now();
  command_received_.store(true);
}

bool SimpleMujocoRos2Interface::getLatestCommand(MujocoCommand & cmd_out)
{
  std::lock_guard<std::mutex> lock(command_mutex_);

  if (!command_received_.load() || !latest_command_.is_valid)
  {
    return false;
  }

  // Check command timeout
  if (isCommandTimedOut())
  {
    return false;
  }

  auto command_age = (this->now() - latest_command_.timestamp).seconds() * 1000.0;  // ms
  if (command_age > MAX_COMMAND_AGE_MS)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Command too old: %.2f ms",
                         command_age);
    return false;
  }

  cmd_out = latest_command_;
  return true;
}

bool SimpleMujocoRos2Interface::isCommandValid() const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return command_received_.load() && latest_command_.is_valid && !isCommandTimedOut();
}

bool SimpleMujocoRos2Interface::isCommandTimedOut() const
{
  auto timeout = (this->now() - last_command_time_).seconds() * 1000.0;  // ms
  return timeout > COMMAND_TIMEOUT_MS;
}

void SimpleMujocoRos2Interface::update()
{
  if (!sim_)
}

void SimpleMujocoRos2Interface::applyJointCommand(const mjModel * m, mjData * d)
{
  MujocoCommand cmd;
  if (!getLatestCommand(cmd))
  {
    return;
  }

  // Validate joint limits
  MujocoCommand validated_cmd = cmd;
  // validateJointLimits(m, validated_cmd);

  // Apply position control if available
  if (!validated_cmd.q_des.empty() && validated_cmd.q_des.size() <= (size_t)m->nq)
  {
    for (size_t i = 0; i < validated_cmd.q_des.size() && i < (size_t)m->nq; ++i)
    {
      d->ctrl[i] = validated_cmd.q_des[i];
    }
  }

  // Apply velocity control if available (assumes velocity actuators)
  if (!validated_cmd.dq_des.empty() && validated_cmd.dq_des.size() <= (size_t)m->nu)
  {
    for (size_t i = 0; i < validated_cmd.dq_des.size() && i < (size_t)m->nu; ++i)
    {
      if (i < (size_t)m->nq)
      {  // position control takes priority
        continue;
      }
      d->ctrl[i] = validated_cmd.dq_des[i];
    }
  }

  // Apply torque control if available
  if (!validated_cmd.tau_des.empty() && validated_cmd.tau_des.size() <= (size_t)m->nu)
  {
    for (size_t i = 0; i < validated_cmd.tau_des.size() && i < (size_t)m->nu; ++i)
    {
      d->ctrl[i] = validated_cmd.tau_des[i];
    }
  }
}

void SimpleMujocoRos2Interface::applyEmergencyStop(const mjModel * m, mjData * d)
{
  // Set all controls to zero for safety
  for (int i = 0; i < m->nu; ++i)
  {
    d->ctrl[i] = 0.0;
  }

  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "Emergency stop applied - no valid command received");
}

// void SimpleMujocoRos2Interface::validateJointLimits(const mjModel * m, MujocoCommand & cmd)
// {
//   // Validate position limits
//   if (!cmd.q_des.empty())
//   {
//     for (size_t i = 0; i < cmd.q_des.size() && i < (size_t)m->nq; ++i)
//     {
//       if (m->jnt_limited[i])
//       {
//         double qmin = m->jnt_range[2 * i];
//         double qmax = m->jnt_range[2 * i + 1];

//         if (cmd.q_des[i] < qmin)
//         {
//           RCLCPP_WARN(this->get_logger(), "Joint %zu position %.3f below limit %.3f", i,
//                       cmd.q_des[i], qmin);
//           cmd.q_des[i] = qmin;
//         }
//         if (cmd.q_des[i] > qmax)
//         {
//           RCLCPP_WARN(this->get_logger(), "Joint %zu position %.3f above limit %.3f", i,
//                       cmd.q_des[i], qmax);
//           cmd.q_des[i] = qmax;
//         }
//       }
//     }
//   }

//   // Validate effort limits (if available in model)
//   if (!cmd.tau_des.empty())
//   {
//     for (size_t i = 0; i < cmd.tau_des.size() && i < (size_t)m->nu; ++i)
//     {
//       // MuJoCo actuator force limits
//       if (i < (size_t)m->nu && m->actuator_forcelimited && m->actuator_forcelimited[i])
//       {
//         double fmin = m->actuator_forcerange[2 * i];
//         double fmax = m->actuator_forcerange[2 * i + 1];

//         if (cmd.tau_des[i] < fmin)
//         {
//           cmd.tau_des[i] = fmin;
//         }
//         if (cmd.tau_des[i] > fmax)
//         {
//           cmd.tau_des[i] = fmax;
//         }
//       }
//     }
//   }
// }

void SimpleMujocoRos2Interface::pubJointState()
{
  if (!sim_)
    return;

  // This will be called by the timer, but actual joint state publishing
  // should be done from the physics thread where we have access to mjModel and mjData
  // For now, we'll keep this as a placeholder
}