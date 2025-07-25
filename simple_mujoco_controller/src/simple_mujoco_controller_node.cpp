#include "simple_mujoco_controller.hpp"

#include <csignal>

volatile sig_atomic_t g_signal_status = 0;

void signalHandler(int signum)
{
  g_signal_status = signum;
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  try
  {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    rclcpp::init(argc, argv);

    const double Hz = 1000;
    auto node = std::make_shared<SimpleMujocoController>(Hz);

    if (!node)
    {
      throw std::runtime_error("Failed to create controller node");
    }

    node->update();

    rclcpp::shutdown();
    return 0;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("simple_mujoco_controller"), "Exception: %s", e.what());
    return 1;
  }
}