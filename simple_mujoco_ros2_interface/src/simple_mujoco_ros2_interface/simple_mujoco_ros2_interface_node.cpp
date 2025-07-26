// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "glfw_adapter.h"

#include "simple_mujoco_ros2_interface.hpp"

using namespace std;

namespace mj = ::mujoco;

static std::unique_ptr<mj::Simulate> sim;  //  simulation pointer

void signalHandler(int signum)
{
  cout << "Interrupt signal (" << signum << ") received." << endl;

  if (sim)
    sim->exitrequest.store(true);  // terminate simulation

  rclcpp::shutdown();  // terminate ROS2 node
}

int main(int argc, char ** argv)
{
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version())
    mju_error("Headers and library have different versions");

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  sim = std::make_unique<mj::Simulate>(std::make_unique<mj::GlfwAdapter>(), &cam, &opt, &pert,
                                       /* is_passive = */ false);

  rclcpp::init(argc, argv);
  const double Hz = 500;
  auto node = std::make_shared<SimpleMujocoRos2Interface>(sim.get(), Hz);

  //* set robot model file path *//
  std::string root = PROJECT_ROOT_DIR;
  std::string model_path = root + "/assets/model/scene.xml";
  const char * filename = model_path.c_str();

  std::thread physics_thread(&SimpleMujocoRos2Interface::physicsThread, node.get(), filename);
  std::thread ros_thread(&SimpleMujocoRos2Interface::spinnerLoop, node.get());

  sim->RenderLoop();  // start simulation UI loop (blocking call)

  if (physics_thread.joinable())
    physics_thread.join();
  if (ros_thread.joinable())
    ros_thread.join();

  rclcpp::shutdown();

  return 0;
}
