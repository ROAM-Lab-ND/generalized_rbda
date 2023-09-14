#!/bin/bash

# TODO(@MatthewChignoli): Two arguments, the first tells which accuracy benchmark to run, the second tells whether not to run the C++ binary

# Flag to determine whether to run the C++ binary (default is 1)
run_cpp=1

# Check if an argument is provided to the script
if [[ $# -gt 0 ]]; then
  # Parse the argument
  run_cpp=$1
else
  echo "Usage: $0 [run_cpp]"
  echo -e "  run_cpp: 0 to skip running the C++ binary, 1 (default) to run it\n\n"
fi

# Run the C++ binary if the flag is set to 1
if [[ $run_cpp -eq 1 ]]; then
    # ./bin/accuracy_benchmark_fd
    # ./bin/accuracy_benchmark_id
    # ./bin/accuracy_benchmark_atf
    ./bin/accuracy_benchmark_robots
fi

# Run MATLAB and execute the script to plot the results
matlab -nodisplay -nosplash -nodesktop -r "try, addpath('../Benchmarking'); accuracy_benchmark_robots; catch ME, disp(ME.message), exit(1), end; exit(0);"
