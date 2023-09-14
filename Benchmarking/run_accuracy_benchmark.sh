#!/bin/bash

# Flag to determine whether to run the C++ binary (default is 1)
benchmark=""

# Check if at least one argument is provided
if [[ $# -eq 1 ]]; then
  # Parse the first argument as the benchmark
  benchmark=$1
else
  echo "Usage: $0 [benchmark]"
  echo -e "  benchmark: Specify which accuracy benchmark to run ('f', 'i', 'a', or 'r')\n"
  exit 1
fi

# Check if the benchmark argument is valid
case "$benchmark" in
  "f" | "F" | "fd" | "FD")
    benchmark="fd"
    ;;
  "i" | "I" | "id" | "ID")
    benchmark="id"
    ;;
  "a" | "A" | "atf" | "ATF")
    benchmark="atf"
    ;;
  "r" | "R" | "robots" | "Robots")
    benchmark="robots"
    ;;
  *)
    echo "Invalid benchmark argument. Choose one of 'f', 'i', 'a', or 'r'."
    exit 1
    ;;
esac

# Run the binary
./bin/accuracy_benchmark_$benchmark

# Run MATLAB and execute the script to plot the results
matlab -nodisplay -nosplash -nodesktop -r "try, addpath('../Benchmarking/plotting'); accuracy_benchmark_$benchmark; catch ME, disp(ME.message), exit(1), end; exit(0);"
