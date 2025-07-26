#include "simple_mujoco_ros2_interface.hpp"

namespace mju = ::mujoco::sample_util;
using namespace std;

using Seconds = std::chrono::duration<double>;

// Static instance pointer definition
SimpleMujocoRos2Interface * SimpleMujocoRos2Interface::instance_ = nullptr;

SimpleMujocoRos2Interface::SimpleMujocoRos2Interface(mj::Simulate * sim, const double Hz)
  : Node("simple_mujoco_ros2_interface"), sim_(sim), loop_rate_(Hz)
{
  instance_ = this;  // set static instance pointer

  /* Initialize simulation UI */
  sim_->ui0_enable = false;        // left UI is disabled (TAB)
  sim_->ui1_enable = false;        // right UI <is disabled (Shift + TAB)
  sim_->pending_.load_key = true;  // load key frame
  sim_->run = false;

  /* Initialize vector size */
  q_mes_.resize(kDoF);
  dq_mes_.resize(kDoF);
  tau_mes_.resize(kDoF);

  mj_cmd_ptr_ = std::make_shared<MujocoCommand>();
  mj_cmd_ptr_->q_des.resize(kDoF);
  mj_cmd_ptr_->dq_des.resize(kDoF);
  mj_cmd_ptr_->tau_des.resize(kDoF);

  /* Define Subscriber */
  const auto qos_jnt_command = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
  sub_joint_cmd_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "mj_joint_command", qos_jnt_command,
    std::bind(&SimpleMujocoRos2Interface::subJointCommand, this, std::placeholders::_1));

  const auto qos_gui = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  sub_gui_cmd_ = this->create_subscription<std_msgs::msg::Bool>(
    "mj_gui_command", qos_gui,
    std::bind(&SimpleMujocoRos2Interface::subGuiCommand, this, std::placeholders::_1));

  /* Define Publisher */
  const auto qos_sim_state = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  pub_sim_state_ = this->create_publisher<std_msgs::msg::Bool>("mj_sim_state", qos_sim_state);

  const auto qos_jnt_state = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
  pub_joint_state_ =
    this->create_publisher<sensor_msgs::msg::JointState>("mj_joint_state", qos_jnt_state);

  RCLCPP_INFO(this->get_logger(), "SimpleMujocoRos2Interface initialized");
}

void SimpleMujocoRos2Interface::subJointCommand(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->name.size() != kDoF || msg->position.size() != kDoF || msg->velocity.size() != kDoF ||
      msg->effort.size() != kDoF)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid number of joints in command message");
    return;
  }

  for (int i = 0; i < kDoF; ++i)
  {
    mj_cmd_ptr_->q_des[i] = msg->position[i];
    mj_cmd_ptr_->dq_des[i] = msg->velocity[i];
    mj_cmd_ptr_->tau_des[i] = msg->effort[i];
  }
}

void SimpleMujocoRos2Interface::subGuiCommand(const std_msgs::msg::Bool::SharedPtr msg)
{
  bIsSimRunTriggered_ = msg->data;
}

void SimpleMujocoRos2Interface::write()
{
  const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
  sim_->run = bIsSimRunTriggered_;  // triggered by GUI command

  switch (mj_cmd_ptr_->mode)
  {
  case 0:  // position control
    for (int i = 0; i < kDoF; ++i)
      d_->qpos[i] = mj_cmd_ptr_->q_des[i];
    break;

  case 1:  // velocity control
    for (int i = 0; i < kDoF; ++i)
      d_->qvel[i] = mj_cmd_ptr_->dq_des[i];
    break;

  case 2:  // torque control
    for (int i = 0; i < kDoF; ++i)
      d_->ctrl[i] = mj_cmd_ptr_->tau_des[i];
    break;
  }
}

void SimpleMujocoRos2Interface::read()
{
  const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
  for (int i = 0; i < kDoF; ++i)
  {
    q_mes_[i] = d_->sensordata[i];
    dq_mes_[i] = d_->sensordata[i + kDoF];
    tau_mes_[i] = d_->sensordata[i + 2 * kDoF];
  }
}

void SimpleMujocoRos2Interface::controlCallback(const mjModel * m, mjData * d)
{
  if (instance_)
  {
    instance_->write();
    instance_->read();
  }
}

void SimpleMujocoRos2Interface::pubMujocoState()
{
  auto msg = sensor_msgs::msg::JointState();
  msg.name.resize(kDoF);
  msg.position.resize(kDoF);
  msg.velocity.resize(kDoF);
  msg.effort.resize(kDoF);

  msg.header.stamp = this->now();
  for (int i = 0; i < kDoF; ++i)
  {
    msg.name[i] = "joint" + std::to_string(i + 1);
    msg.position[i] = q_mes_[i];
    msg.velocity[i] = dq_mes_[i];
    msg.effort[i] = tau_mes_[i];
  }
  pub_joint_state_->publish(msg);
}

mjModel * SimpleMujocoRos2Interface::loadModel(const char * file)
{
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);  // this copy is needed so that mju::strlen call below compiles
  RCLCPP_INFO(this->get_logger(), "Load model from: %s\n", filename);

  if (!filename[0])  // make sure filename is not empty
    return nullptr;

  // load and compile
  const int error_length = 1024;  // load error string length
  char loadError[error_length] = "";
  mjModel * mnew = 0;
  if (mju::strlen_arr(filename) > 4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4))
  {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew)
      mju::strcpy_arr(loadError, "could not load binary model");
  }
  else
  {
    mnew = mj_loadXML(filename, nullptr, loadError, error_length);

    if (loadError[0])
    {  // remove trailing newline character from loadError
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length - 1] == '\n')
        loadError[error_length - 1] = '\0';
    }
  }

  mju::strcpy_arr(sim_->load_error, loadError);

  if (!mnew)
  {
    std::printf("%s\n", loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0])
  {  // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim_->run = 0;
  }

  return mnew;
}

void SimpleMujocoRos2Interface::physicsThread(const char * filename)
{
  if (filename != nullptr)
  {
    sim_->LoadMessage(filename);
    m_ = this->loadModel(filename);
    if (m_)
    {
      const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
      d_ = mj_makeData(m_);
    }

    if (d_)
    {
      sim_->Load(m_, d_, filename);

      const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
      mj_forward(m_, d_);
    }
    else
    {
      sim_->LoadMessageClear();
    }
  }
  mjcb_control = &SimpleMujocoRos2Interface::controlCallback;
  this->physicsLoop();

  mj_deleteData(d_);
  mj_deleteModel(m_);
}

void SimpleMujocoRos2Interface::physicsLoop()
{
  // cpu-sim_ syncronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  const double sync_misalign = 0.1;  // maximum mis-alignment before re-sync (simulation seconds)
  const double sim_refresh_fraction = 0.7;  // fraction of refresh available for simulation

  while (!sim_->exitrequest.load())
  {
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);

    if (m_)
    {  // run only if model is present
      if (sim_->run)
      {
        bool stepped = false;

        const auto startCPU = mj::Simulate::Clock::now();  // record cpu time at start of iteration
        const auto elapsedCPU = startCPU - syncCPU;        // elapsed CPU  since last sync
        double elapsedSim = d_->time - syncSim;            // simulation time since last sync

        // requested slow-down factor
        double slowdown = 100 / sim_->percentRealTime[sim_->real_time_index];

        // misalignment condition: distance from target sim_ time is bigger than syncmisalign
        bool misaligned =
          mju_abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > sync_misalign;

        if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
            misaligned || sim_->speed_changed)
        {  // out-of-sync (for any reason): reset sync times, step
          // re-sync
          syncCPU = startCPU;
          syncSim = d_->time;
          sim_->speed_changed = false;

          // run single step, let next iteration deal with timing
          mj_step(m_, d_);
          stepped = true;
        }
        else  // in-sync: step until ahead of cpu
        {
          bool measured = false;
          mjtNum prevSim = d_->time;

          double refreshTime = sim_refresh_fraction / sim_->refresh_rate;

          while (Seconds((d_->time - syncSim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                 mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime))
          {  // step while sim_ lags behind cpu and within refreshTime
            if (!measured && elapsedSim)
            {  // measure slowdown before first step
              sim_->measured_slowdown =
                std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
              measured = true;
            }

            mj_step(m_, d_);
            stepped = true;

            if (d_->time < prevSim)  // break if reset
              break;
          }
        }

        if (stepped)  // save current state to history buffer
          sim_->AddToHistory();
      }
      else  // paused
      {     // run mj_forward, to update rendering and joint sliders
        mj_forward(m_, d_);
        sim_->speed_changed = true;
      }
    }
  }
}

void SimpleMujocoRos2Interface::spinnerLoop()
{
  auto msg = std_msgs::msg::Bool();

  while (rclcpp::ok())
  {
    rclcpp::spin_some(this->shared_from_this());
    {
      const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
      bIsSimRunning_ = sim_->run;
    }
    msg.data = bIsSimRunning_;
    pub_sim_state_->publish(msg);

    this->pubMujocoState();
    loop_rate_.sleep();
  }
}
