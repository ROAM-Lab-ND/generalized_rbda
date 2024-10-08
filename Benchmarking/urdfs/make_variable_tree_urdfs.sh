#!/bin/bash

# Initialize variables
arg1=("1" "2" "4" "6") # revolute rotor branch: branch count
arg2=("1" "2" "3" "4" "5" "6" "7" "8" "9" "10") # revolute rotor branch: depth count
arg3=("1" "2" "4" "6") # revolute rotor pair branch: branch count
arg4=("1" "2" "3" "4" "5" "6" "7") # revolute rotor pair branch: depth count
arg5=("1" "2" "4" "6") # four bar branch: branch count
arg6=("1" "2" "3" "4" "5" "6" "7" "8" "9" "10") # four bar branch: depth count

# Loop through arguments
for arg in "$@"
do
    case $arg in
        -b1=*|--revolute-rotor-branch-list=*)
        IFS=',' read -r -a arg1 <<< "${arg#*=}"
        shift
        ;;
        -d1=*|--revolute-rotor-depth-list=*)
        IFS=',' read -r -a arg2 <<< "${arg#*=}"
        shift
        ;;
        -b2=*|--revolute-rotor-pair-branch-list=*)
        IFS=',' read -r -a arg3 <<< "${arg#*=}"
        shift
        ;;
        -d2=*|--revolute-rotor-pair-depth-list=*)
        IFS=',' read -r -a arg4 <<< "${arg#*=}"
        shift
        ;;
        -b3=*|--four-bar-branch-list=*)
        IFS=',' read -r -a arg5 <<< "${arg#*=}"
        shift
        ;;
        -d3=*|--four-bar-depth-list=*)
        IFS=',' read -r -a arg6 <<< "${arg#*=}"
        shift
        ;;
        *)
        echo "Invalid argument: $arg" 1>&2
        exit 1
        ;;
    esac
done

# Output the results
echo "[revolute_rotor_branch] branch count: ${arg1[@]}"
echo "[revolute_rotor_branch] depth count: ${arg2[@]}"
echo "[revolute_rotor_pair_branch] branch count: ${arg3[@]}"
echo "[revolute_rotor_pair_branch] depth count: ${arg4[@]}"
echo "[four_bar_branch] branch count: ${arg5[@]}"
echo "[four_bar_branch] depth count: ${arg6[@]}"

# get the current directory
CURRENTDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# xacros all robots
cd $CURRENTDIR

VAR_REVOLUTE_DIR="variable_revolute_urdf"
VAR_REVOLUTE_PAIR_DIR="variable_revolute_pair_urdf"
VAR_FOUR_BAR_DIR="variable_four_bar_urdf"

rm -rf "$VAR_REVOLUTE_DIR"
rm -rf "$VAR_REVOLUTE_PAIR_DIR"
rm -rf "$VAR_FOUR_BAR_DIR"
mkdir "$VAR_REVOLUTE_DIR"
mkdir "$VAR_REVOLUTE_PAIR_DIR"
mkdir "$VAR_FOUR_BAR_DIR"

for branch_count in "${arg1[@]}"; do
    for depth_count in "${arg2[@]}"; do
        xacro revolute_rotor_branch.xacro branch_count:=${branch_count} depth_count:=${depth_count} > \
            "$VAR_REVOLUTE_DIR/revolute_rotor_branch_${branch_count}_${depth_count}.urdf"
    done
done

for branch_count in "${arg3[@]}"; do
    for depth_count in "${arg4[@]}"; do
        xacro revolute_rotor_pair_branch.xacro branch_count:=${branch_count} depth_count:=${depth_count} > \
            "$VAR_REVOLUTE_PAIR_DIR/revolute_rotor_pair_branch_${branch_count}_${depth_count}.urdf"
    done
done

for branch_count in "${arg5[@]}"; do
    for depth_count in "${arg6[@]}"; do
        xacro four_bar_branch.xacro branch_count:=${branch_count} depth_count:=${depth_count} > \
            "$VAR_FOUR_BAR_DIR/four_bar_branch_${branch_count}_${depth_count}.urdf"
    done
done
