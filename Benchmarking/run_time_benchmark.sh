#!/bin/bash

# Run the binary
./bin/time_benchmark

# Run MATLAB and execute the script to plot the results
matlab -nodisplay -nosplash -nodesktop -r "try, addpath('../Benchmarking'); time_benchmark; catch ME, disp(ME.message), exit(1), end; exit(0);"
