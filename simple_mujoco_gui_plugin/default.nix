# This defines a function which returns a packge, intended to be evaluated by callPackage
{ lib
, buildRosPackage
, pluginlib
, std-msgs
, sensor-msgs
, qt-gui-core
, rqt-gui
, rqt-gui-py
, rqt-gui-cpp
, qtcreator
, qt5Full
}:

buildRosPackage {
  pname = "simple-mujoco-gui-plugin";
  version = "0.0.0"; 
  src = ./.;
  
  buildType = "ament_cmake";

  propagatedBuildInputs = [
    pluginlib
    std-msgs
    sensor-msgs
    qt-gui-core
    rqt-gui
    rqt-gui-py
    rqt-gui-cpp
    qtcreator
    qt5Full
  ];  
  
  meta = {
    description = "RQt GUI plugin for MuJoCo UI Interaction";
  };
}
