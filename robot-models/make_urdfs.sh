#!/bin/bash

link_count=${1:-3}

# get the current directory
CURRENTDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# xacros all robots
cd $CURRENTDIR

VAR_REVOLUTE_DIR="variable_revolute_urdf"
mkdir "$VAR_REVOLUTE_DIR"
rm "$VAR_REVOLUTE_DIR"/*
for (( i=link_count; i>0; i-- ))
do
    xacro revolute_rotor_parallel.xacro loop_count:=$i > "$VAR_REVOLUTE_DIR/revolute_rotor_parallel_${i}.urdf"
    xacro revolute_rotor_chain.xacro loop_count:=$i > "$VAR_REVOLUTE_DIR/revolute_rotor_chain_${i}.urdf"
done

xacro mini_cheetah.xacro > mini_cheetah.urdf
xacro mini_cheetah_fr_leg.xacro > mini_cheetah_fr_leg.urdf
xacro mini_cheetah_fl_leg.xacro > mini_cheetah_fl_leg.urdf
xacro mini_cheetah_hr_leg.xacro > mini_cheetah_hr_leg.urdf
xacro mini_cheetah_hl_leg.xacro > mini_cheetah_hl_leg.urdf
xacro mit_humanoid_leg.xacro > mit_humanoid_leg.urdf
xacro mit_humanoid.xacro > mit_humanoid.urdf
