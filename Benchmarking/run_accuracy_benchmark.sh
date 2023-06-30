#!/bin/bash

# Run the binary
./bin/accuracy_benchmark

# Run MATLAB and execute the script to plot the results
matlab -nodisplay -nosplash -nodesktop -r "try, addpath('../Benchmarking'); accuracy_benchmark; catch ME, disp(ME.message), exit(1), end; exit(0);"
