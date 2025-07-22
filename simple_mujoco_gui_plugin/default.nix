# This defines a function which returns a packge, intended to be evaluated by callPackage
{ lib
, buildRosPackage
, pluginlib
, std-msgs
, sensor-msgs
, geometry-msgs
, qt-gui-core
, rqt-gui
, rqt-gui-py
, rqt-gui-cpp
, qtcreator
, qt5Full

# , mylib
# , elmo-msgs
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
    geometry-msgs
    qt-gui-core
    rqt-gui
    rqt-gui-py
    rqt-gui-cpp
    qtcreator
    qt5Full

    # mylib
    # elmo-msgs
  ];  
  
  meta = {
    description = "RQt GUI node for MuJoCo UI Interaction";
  };
}
