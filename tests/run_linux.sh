#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
c++ -std=c++17 -O2 -Wall -Wextra -pedantic tests/engine_test.cpp -o tests/engine_test
./tests/engine_test
