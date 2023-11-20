#!/bin/bash

CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
GRBDA_DIR=${CURRENT_DIR}/../..
cd $GRBDA_DIR
mkdir grbda-dependencies
cd grbda-dependencies
DEP_PATH=$(pwd)

# Required packages:
sudo apt install \
    mesa-common-dev freeglut3-dev libblas-dev \
    liblapack-dev gfortran gcc build-essential \
    libglib2.0-dev libusb-1.0-0-dev libsdl2-dev -y

# Install CMake 3.15.0
echo "[CMAKE] BUILDING CMAKE 3.15.0..."
if command -v cmake &> /dev/null
then
    sudo apt remove cmake
    sudo apt purge --auto-remove cmake
    echo "cmake has been uninstalled."
else
    echo "cmake is not installed."
fi
sudo apt-get install build-essential
wget http://www.cmake.org/files/v3.15/cmake-3.15.0.tar.gz
tar xf cmake-3.15.0.tar.gz
rm cmake-3.15.0.tar.gz
cd cmake-3.15.0
./configure
make
sudo make install 
hash -r
cmake --version 
echo "[CMAKE] CMAKE installed"

# Eigen 3.4.0 Installation
echo "[EIGEN] BUILDING EIGEN 3.4.0..."
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
unzip eigen-3.4.0.zip
rm eigen-3.4.0.zip
cd eigen-3.4.0
mkdir build && cd build && cmake .. && sudo make -j8 && sudo make install 
cd $DEP_PATH 
echo "[EIGEN] EIGEN 3.4.0 INSTALLED"

# Casadi 3.6.3 Installation
echo "[CASADI] BUILDING CASADI 3.6.3..."
mkdir -p casadi
cd casadi
wget https://github.com/casadi/casadi/releases/download/3.6.3/casadi-source-v3.6.3.zip
unzip -q casadi-source-v3.6.3.zip
rm casadi-source-v3.6.3.zip
mkdir build && cd build && cmake .. 
sudo make && sudo make install
echo "[CASADI] CASADI 3.6.3 INSTALLED"
