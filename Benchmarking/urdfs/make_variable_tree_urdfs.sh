#!/bin/bash

# Initialize variables
arg1="4" # revolute rotor serial/parallel loop count
arg2=("1" "2" "4" "6") # revolute rotor branch: branch count
arg3=("1" "2" "3" "4" "5" "6" "7" "8" "9" "10") # revolute rotor branch: depth count
arg4=("1" "2" "4" "6") # revolute rotor pair branch: branch count
arg5=("1" "2" "3" "4" "5" "6" "7") # revolute rotor pair branch: depth count
arg6=("1" "2" "4" "6") # four bar branch: branch count
arg7=("1" "2" "3" "4" "5" "6" "7" "8" "9" "10") # four bar branch: depth count

# Loop through arguments
for arg in "$@"
do
    case $arg in
        -sp=*|--serial-parallel=*)
        arg1="${arg#*=}"
        shift
        ;;
        -b1=*|--revolute-rotor-branch-list=*)
        IFS=',' read -r -a arg2 <<< "${arg#*=}"
        shift
        ;;
        -d1=*|--revolute-rotor-depth-list=*)
        IFS=',' read -r -a arg3 <<< "${arg#*=}"
        shift
        ;;
        -b2=*|--revolute-rotor-pair-branch-list=*)
        IFS=',' read -r -a arg4 <<< "${arg#*=}"
        shift
        ;;
        -d2=*|--revolute-rotor-pair-depth-list=*)
        IFS=',' read -r -a arg5 <<< "${arg#*=}"
        shift
        ;;
        -b3=*|--four-bar-branch-list=*)
        IFS=',' read -r -a arg6 <<< "${arg#*=}"
        shift
        ;;
        -d3=*|--four-bar-depth-list=*)
        IFS=',' read -r -a arg7 <<< "${arg#*=}"
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
echo "[revolute_rotor_pair_branch] branch count: ${arg4[@]}"
echo "[revolute_rotor_pair_branch] depth count: ${arg5[@]}"
echo "[four_bar_branch] branch count: ${arg6[@]}"
echo "[four_bar_branch] depth count: ${arg7[@]}"

# get the current directory
CURRENTDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# xacros all robots
cd $CURRENTDIR

VAR_REVOLUTE_DIR="variable_revolute_urdf"
VAR_REVOLUTE_PAIR_DIR="variable_revolute_pair_urdf"
VAR_FOUR_BAR_DIR="variable_four_bar_urdf"
mkdir "$VAR_REVOLUTE_DIR"
mkdir "$VAR_REVOLUTE_PAIR_DIR"
mkdir "$VAR_FOUR_BAR_DIR"
rm "$VAR_REVOLUTE_DIR"/*
rm "$VAR_REVOLUTE_PAIR_DIR"/*
rm "$VAR_FOUR_BAR_DIR"/*
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
for branch_count in "${arg4[@]}"; do
    for depth_count in "${arg5[@]}"; do
        xacro revolute_rotor_pair_branch.xacro branch_count:=${branch_count} depth_count:=${depth_count} > \
            "$VAR_REVOLUTE_PAIR_DIR/revolute_rotor_pair_branch_${branch_count}_${depth_count}.urdf"
    done
done
for branch_count in "${arg6[@]}"; do
    for depth_count in "${arg7[@]}"; do
        xacro four_bar_branch.xacro branch_count:=${branch_count} depth_count:=${depth_count} > \
            "$VAR_FOUR_BAR_DIR/four_bar_branch_${branch_count}_${depth_count}.urdf"
    done
done