#!/bin/bash

if [  $# -eq 1 ]; then
    matlab -nodisplay -nosplash -nodesktop -r "try, $1; catch ME, disp(ME.message), exit(1), end; exit(0);"
else
    echo "Usage: $0 [target]"
    echo -e "  target: Matlab script to be run\n"
    exit 1
fi
