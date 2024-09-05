#!/bin/bash

# Get the current directory
CURRENTDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# Define arrays for depths and corresponding loop sizes for explicit parallel chains
depths=("depth5" "depth10" "depth20" "depth40")
loop_sizes=(
  "2 4 6 8 10"      # For depth5
  "2 4 8 16 20"     # For depth10
  "2 4 10 20 40"    # For depth20
  "2 4 20 40 80"    # For depth40
)

# Change to the appropriate directory
cd "$CURRENTDIR/parallel_chains/Explicit" || { echo "Directory not found: $CURRENTDIR/parallel_chains/Explicit"; exit 1; }

# Loop through each depth
for i in "${!depths[@]}"; do
  depth="${depths[$i]}"
  cd "$depth" || { echo "Directory not found: $depth"; exit 1; }
  
  # Loop through the loop sizes associated with this depth
  for size in ${loop_sizes[$i]}; do
    xacro "loop_size${size}.xacro" > "loop_size${size}.urdf"
  done
  
  cd ..
done

# Define arrays for depths and corresponding loop sizes for implicit parallel chains
depths=("depth5")
loop_sizes=(
  "3 5 7 9 11"      # For depth5
)

# Change to the appropriate directory
cd "$CURRENTDIR/parallel_chains/Implicit" || { echo "Directory not found: $CURRENTDIR/parallel_chains/Implicit"; exit 1; }

# Loop through each depth
for i in "${!depths[@]}"; do
  depth="${depths[$i]}"
  cd "$depth" || { echo "Directory not found: $depth"; exit 1; }
  
  # Loop through the loop sizes associated with this depth
  for size in ${loop_sizes[$i]}; do
    xacro "loop_size${size}.xacro" > "loop_size${size}.urdf"
  done
  
  cd ..
done
