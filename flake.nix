{
  description = "Simple MuJoCo ROS2 packages including controller, GUI, and simulation";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # ! IMPORTANT 
    nixgl.url = "github:nix-community/nixGL";
  };

  outputs = { self, nixpkgs, nix-ros-overlay, nixgl }:
    let
      system = "x86_64-linux";
      
      overlays = [
        nix-ros-overlay.overlays.default
        nixgl.overlay
      ];
      
      pkgs = import nixpkgs {
        inherit system overlays;
      };

      rospkgs = pkgs.rosPackages.humble;
      rosPackages = with rospkgs; [
        rclcpp
        ros-core
        ament-cmake-core

        std-msgs
        sensor-msgs

        joy
      ];

      # mylib = pkgs.callPackage ./mylib {};
      # elmo-msgs = rospkgs.callPackage ./elmo_msgs {};

      simple-mujoco-gui-plugin = rospkgs.callPackage ./simple_mujoco_gui_plugin {};

      guiPackages = [
        simple-mujoco-gui-plugin
        pkgs.nixgl.nixGLIntel
      ];
      
    in
      {
        devShells.${system} = {
          default = pkgs.mkShell {
            packages = [
              pkgs.colcon

              # ... other non-ROS packages
              pkgs.eigen
              
              (rospkgs.buildEnv {
                paths = rosPackages;
              })
            ];
          };

          sim = pkgs.mkShell {
            shellHook = ''
              alias start_sim="nixGLIntel ./simple_mujoco_ros2_interface"
              export PS1="(sim) $PS1"
            '';

            packages = [
              pkgs.mujoco 
              pkgs.glfw
              pkgs.nixgl.nixGLIntel
              (rospkgs.buildEnv {
                paths = rosPackages;
              })
            ];
          };

          gui = pkgs.mkShell {
            shellHook = ''
              alias start_gui="nixGLIntel ros2 run simple_mujoco_gui_plugin simple_mujoco_gui_plugin"
              export PS1="(gui) $PS1"
            '';

            packages = [
              (rospkgs.buildEnv {
                paths = rosPackages
                        ++ guiPackages;
              })
            ];
          };
        };
      };

  nixConfig = {
    bash-prompt-prefix = "\\[\\e[38;5;12m\\](nix)\\[\\e[38;5;220m\\] ";

    extra-substituters = [
      "https://ros.cachix.org"
      "https://nix-community.cachix.org"
    ];
    
    extra-trusted-public-keys = [
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
      "nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs="
    ];
  };
}
