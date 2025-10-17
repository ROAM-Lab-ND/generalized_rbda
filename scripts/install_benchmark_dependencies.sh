#!/bin/bash

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
GRBDA_DIR=${CURRENT_DIR}/..
cd $GRBDA_DIR
mkdir grbda-dependencies
cd grbda-dependencies
DEP_PATH=$(pwd)

# Console Bridge
cd $DEP_PATH 
wget https://github.com/ros/console_bridge/archive/refs/tags/1.0.2.zip
unzip 1.0.2.zip
cd console_bridge-1.0.2
mkdir build && cd build
cmake ..
make
sudo make install

# URDFDOM_HEADERS
cd $DEP_PATH
git clone https://github.com/mit-biomimetics/urdfdom_headers.git
cd urdfdom_headers
git checkout 632d1c7d13080b57c39f71bf4be4bbcaa337ec7c
mkdir build && cd build
cmake ..
make
sudo make install

# URDFDOM
cd $DEP_PATH
git clone https://github.com/mit-biomimetics/urdfdom.git
cd urdfdom
git checkout 0425417a68c331ae308a86fc752474ee55d7b041
mkdir build && cd build
cmake ..
make
sudo make install

#URDF Parser
cd $DEP_PATH
git clone https://github.com/mit-biomimetics/URDF-Parser.git
cd URDF-Parser
git checkout 2b020ae4ca0cf1c2e738cabbc0721a1652c7b4d7
mkdir build && cd build
cmake ..
make 
sudo make install

cd $DEP_PATH
git clone https://github.com/jrl-umi3218/jrl-cmakemodules.git
cd jrl-cmakemodules
git checkout 2dd858f5a71d8224f178fb3dc0bcd95256ba10e7
mkdir build && cd build
cmake ..
make
sudo make install

sudo apt-get install -y libtinyxml-dev libboost-filesystem-dev libboost-serialization-dev libboost-system-dev libboost-test-dev


# Pinocchio
cd $DEP_PATH
wget https://github.com/stack-of-tasks/pinocchio/archive/refs/tags/v3.3.0.zip
unzip v3.3.0.zip
cd pinocchio-3.3.0
mkdir build && cd build
cmake -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_WITH_URDF_SUPPORT=ON -DBUILD_TESTING=OFF -DBUILD_WITH_CASADI_SUPPORT=ON ..
make
sudo make install
