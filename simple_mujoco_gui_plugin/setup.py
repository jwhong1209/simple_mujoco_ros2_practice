#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["simple_mujoco_gui_plugin"],
    package_dir={"simple_mujoco_gui_plugin": "src/simple_mujoco_gui_plugin"},
    scripts=["scripts/simple_mujoco_gui_plugin"],
)

setup(**d)
