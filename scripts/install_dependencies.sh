#!/bin/bash

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
GRBDA_DIR=${CURRENT_DIR}/..
cd $GRBDA_DIR
mkdir grbda-dependencies
cd grbda-dependencies
DEP_PATH=$(pwd)

# Required packages:
sudo apt install -y \
    mesa-common-dev freeglut3-dev libblas-dev \
    liblapack-dev gfortran gcc build-essential \
    libglib2.0-dev libusb-1.0-0-dev libsdl2-dev \
    libboost-all-dev # for Pinocchio

# Eigen 3.4.0 Installation
cd $DEP_PATH 
echo "[EIGEN] BUILDING EIGEN 3.4.0..."
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
unzip eigen-3.4.0.zip
rm eigen-3.4.0.zip
cd eigen-3.4.0
mkdir build && cd build && cmake .. && sudo make -j8 && sudo make install 
echo "[EIGEN] EIGEN 3.4.0 INSTALLED"

# Casadi 3.6.3 Installation
cd $DEP_PATH 
echo "[CASADI] BUILDING CASADI 3.6.3..."
mkdir -p casadi
cd casadi
wget https://github.com/casadi/casadi/releases/download/3.6.3/casadi-source-v3.6.3.zip
unzip -q casadi-source-v3.6.3.zip
rm casadi-source-v3.6.3.zip
mkdir build && cd build && cmake .. 
sudo make && sudo make install
echo "[CASADI] CASADI 3.6.3 INSTALLED"

# Console Bridge
cd $DEP_PATH
git clone https://github.com/ros/console_bridge.git
cd console_bridge
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make
sudo make install
sudo ldconfig

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

# Pinocchio
cd $DEP_PATH
git clone https://github.com/stack-of-tasks/pinocchio.git
cd pinocchio
mkdir build && cd build
cmake -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_WITH_URDF_SUPPORT=ON -DBUILD_WITH_CASADI_SUPPORT=ON -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF ..
make
sudo make install
