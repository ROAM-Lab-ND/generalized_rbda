#!/bin/bash
run-parts --regex '_benchmark.sh' ../Benchmarking
run-parts --regex '_analysis.sh' ../Benchmarking
