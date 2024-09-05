#!/bin/bash

# Initialize variables
arg1="4" # revolute rotor serial/parallel loop count
arg2=("1" "2" "4" "6") # revolute rotor branch: branch count
arg3=("1" "2" "3" "4" "5" "6" "7" "8" "9" "10") # revolute rotor branch: depth count

# Loop through arguments
for arg in "$@"
do
    case $arg in
        -sp=*|--serial-parallel=*)
        arg1="${arg#*=}"
        shift
        ;;
        -b=*|--branch_list=*)
        IFS=',' read -r -a arg2 <<< "${arg#*=}"
        shift
        ;;
        -d=*|--depth_list=*)
        IFS=',' read -r -a arg3 <<< "${arg#*=}"
        shift
        ;;
        *)
        echo "Invalid argument: $arg" 1>&2
        exit 1
        ;;
    esac
done

# Output the results
echo "[revolute_rotor_chain/parallel] loop count: $arg1"
echo "[revolute_rotor_branch] branch count: ${arg2[@]}"
echo "[revolute_rotor_branch] depth count: ${arg3[@]}"

# get the current directory
CURRENTDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# xacros all robots
cd $CURRENTDIR

VAR_REVOLUTE_DIR="variable_revolute_urdf"
mkdir "$VAR_REVOLUTE_DIR"
rm "$VAR_REVOLUTE_DIR"/*
for (( i=arg1; i>0; i-- ))
do
    xacro revolute_rotor_parallel.xacro loop_count:=$i > "$VAR_REVOLUTE_DIR/revolute_rotor_parallel_${i}.urdf"
    xacro revolute_rotor_chain.xacro loop_count:=$i > "$VAR_REVOLUTE_DIR/revolute_rotor_chain_${i}.urdf"
done
for branch_count in "${arg2[@]}"; do
    for depth_count in "${arg3[@]}"; do
        xacro revolute_rotor_branch.xacro branch_count:=${branch_count} depth_count:=${depth_count} > \
            "$VAR_REVOLUTE_DIR/revolute_rotor_branch_${branch_count}_${depth_count}.urdf"
    done
done

xacro mini_cheetah.xacro > mini_cheetah.urdf
xacro mini_cheetah_fr_leg.xacro > mini_cheetah_fr_leg.urdf
xacro mini_cheetah_fl_leg.xacro > mini_cheetah_fl_leg.urdf
xacro mini_cheetah_hr_leg.xacro > mini_cheetah_hr_leg.urdf
xacro mini_cheetah_hl_leg.xacro > mini_cheetah_hl_leg.urdf
xacro mit_humanoid_leg.xacro > mit_humanoid_leg.urdf
xacro mit_humanoid.xacro > mit_humanoid.urdf