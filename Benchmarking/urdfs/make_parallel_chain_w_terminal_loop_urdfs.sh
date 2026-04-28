#!/bin/bash

# Get the current directory
CURRENTDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# Define loop sizes for implicit parallel chains with terminal loops
loop_sizes=(
  "3 5 11" 
)

# Change to the appropriate directory
cd "$CURRENTDIR/parallel_chains/ImplicitTerminalEnd" || { echo "Directory not found: $CURRENTDIR/parallel_chains/ImplicitTerminalEnd"; exit 1; }

# Loop through each depth
  
# Loop through the loop sizes associated with this depth
for size in ${loop_sizes[$i]}; do
  xacro "loop_size${size}.xacro" > "loop_size${size}.urdf"
done