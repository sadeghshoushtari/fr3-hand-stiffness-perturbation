#!/bin/bash
# Perturbation program for the MOSAIC calibration pipeline (robot 169.254.105.210). Builds perturbation_test first
# if it is missing or older than its source. Extra options are passed on (e.g. --console, another IP).
set -e
here=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
if [ ! -x "$here/perturbation_test" ] || [ "$here/perturbation_test.cpp" -nt "$here/perturbation_test" ]; then
  echo "building perturbation_test ..."
  g++ -std=c++17 -O2 -Wall -Wextra "$here/perturbation_test.cpp" -I/usr/include/eigen3 -o "$here/perturbation_test" \
    -lfranka -pthread
fi
cd "$here"
exec ./perturbation_test "$@"
