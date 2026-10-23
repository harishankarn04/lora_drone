#!/bin/sh
# Build and run the link simulation. Optional args: <hop_ms> <tx_ms> (measured on hardware).
cd "$(dirname "$0")" && c++ -std=c++17 -O2 -Wall -o link_sim link_sim.cpp && ./link_sim "$@"
