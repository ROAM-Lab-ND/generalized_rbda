#!/bin/bash

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
GRBDA_DIR=${CURRENT_DIR}/..
cd $GRBDA_DIR
mkdir grbda-dependencies
cd grbda-dependencies
DEP_PATH=$(pwd)

# Console Bridge
cd $DEP_PATH 
git clone https://github.com/ros/console_bridge.git
cd console_bridge
mkdir build && cd build
cmake ..
make
sudo make install

# URDFDOM_HEADERS
cd $DEP_PATH 
git clone https://github.com/ros/urdfdom_headers.git
cd urdfdom_headers
mkdir build && cd build
cmake ..
make
sudo make install

# URDFDOM
cd $DEP_PATH
git clone https://github.com/ros/urdfdom.git
cd urdfdom
mkdir build && cd build
cmake ..
make
sudo make install

# Pinocchio
cd $DEP_PATH
git clone https://github.com/stack-of-tasks/pinocchio.git
cd pinocchio
mkdir build && cd build
cmake -DBUILD_PYTHON_INTERFACE=OFF ..
make
sudo make install
