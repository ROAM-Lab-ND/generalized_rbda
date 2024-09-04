#!/bin/bash

link_count=${1:-5}
branch_count=${2:-4}
depth_count=${3:-2}

# get the current directory
CURRENTDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# xacros all robots
cd $CURRENTDIR/parallel_chains/explicit

# TODO(@MatthewChignoli): room for automation here
cd depth5
xacro loop_size2.xacro > loop_size2.urdf
xacro loop_size4.xacro > loop_size4.urdf
xacro loop_size6.xacro > loop_size6.urdf
xacro loop_size8.xacro > loop_size8.urdf
xacro loop_size10.xacro > loop_size10.urdf

cd ../depth10
xacro loop_size2.xacro > loop_size2.urdf
xacro loop_size4.xacro > loop_size4.urdf
xacro loop_size8.xacro > loop_size8.urdf
xacro loop_size16.xacro > loop_size16.urdf
xacro loop_size20.xacro > loop_size20.urdf

cd ../depth20
xacro loop_size2.xacro > loop_size2.urdf
xacro loop_size4.xacro > loop_size4.urdf
xacro loop_size10.xacro > loop_size10.urdf
xacro loop_size20.xacro > loop_size20.urdf
xacro loop_size40.xacro > loop_size40.urdf


