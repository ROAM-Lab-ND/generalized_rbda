#!/bin/bash

# Flag to determine whether to run the C++ binary (default is 1)
benchmark=""

# Check if at least one argument is provided
if [[ $# -eq 1 ]]; then
  # Parse the first argument as the benchmark
  benchmark=$1
else
  echo "Usage: $0 [benchmark]"
  echo -e "  benchmark: Specify which time benchmark to run ('f', 'i', or 'r')\n"
  echo -e "  f: Forward Dynamics\n"
  echo -e "  i: Inverse Operational-Space Inertia Matrix\n"
  echo -e "  r: Robots\n"
  exit 1
fi

# Check if the benchmark argument is valid
case "$benchmark" in
  "f" | "F" | "fd" | "FD")
    benchmark="fd"
    ;;
  "i" | "I" | "iosim" | "IOSIM")
    benchmark="iosim"
    ;;
  "r" | "R" | "robots" | "Robots")
    benchmark="robots"
    ;;
  *)
    echo "Invalid benchmark argument. Choose one of 'f', 'i', or 'r'."
    exit 1
    ;;
esac

# Run the binary
./bin/time_benchmark_$benchmark

# Run MATLAB and execute the script to plot the results
matlab -nodisplay -nosplash -nodesktop -r "try, addpath('../Benchmarking/plotting'); time_benchmark_$benchmark; catch ME, disp(ME.message), exit(1), end; exit(0);"
