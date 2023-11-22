#!/bin/bash

# Navigate to the correct directory
SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
MATLAB_DIR="${SCRIPTS_DIR}/matlab"

# Throw an error if the folder matlab/casadi does not exist
if [ ! -d "${MATLAB_DIR}/casadi" ]; then
  echo "Error: ${MATLAB_DIR}/casadi folder does not exist. Get binaries from https://web.casadi.org/get/ first."
  exit 1
fi

# Check if at least one argument is provided
if [[ $# -eq 1 ]]; then
  # Parse the first argument as the target
  target=$1
else
  echo "Usage: $0 [target]"
  echo -e "  target: Specify which codegen targets to run\n"
  echo -e "  all: All codegen scripts\n"
  echo -e "  tello: Tello-related scripts\n"
  echo -e "  rev: Lagrangian-based dynamics of serial robot w/ rev joints\n"
  echo -e "  rev_pair: Lagrangian-based dynamics of serial robot w/ rev pair joints\n"
  exit 1
fi

# Check if the benchmark argument is valid
cd ${MATLAB_DIR}
case "$1" in
  "all")
    ${SCRIPTS_DIR}/run_matlab_script.sh "codegen_tello;codegen_revolute_with_rotor;codegen_revolute_pair_with_rotor"
    ;;
  "tello")
    ${SCRIPTS_DIR}/run_matlab_script.sh "codegen_tello"
    ;;
  "rev")
    ${SCRIPTS_DIR}/run_matlab_script.sh "codegen_revolute_with_rotor"
    ;;
  "rev_pair")
    ${SCRIPTS_DIR}/run_matlab_script.sh "codegen_revolute_pair_with_rotor"
    ;;
  *)
    echo "Invalid target argument."
    exit 1
    ;;
esac
