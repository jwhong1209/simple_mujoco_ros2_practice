# How to Build & Run
```bash
cd ~/<your-workspace>
git clone https://github.com/jwhong1209/simple_mujoco_ros2_practice.git
cd simple_mujoco_ros2_practice
```

- Terminal 1 : Controller package
```bash
nix develop
cd simple_mujoco_controller
mkdir -p build
cd build
cmake ..
make 
./simple_mujoco_controller
```

- Terminal 2 : GUI
```bash
nix develop .#gui
start_gui
  ```

- Terminal 3: MuJoCo package
  - Simulation starts when checkbox in GUI is triggered
```bash
nix develop .#sim
cd simple_mujoco_ros2_interface
mkdir -p build
cd build
cmake ..
make 
start_sim
```

- Terminal 4 : Joy node (optional)
  - Change trajectory type as TELEOP in controller package
```bash
nix develop
ros2 run joy joy_node
```
