#!/bin/bash

echo "Running..."
ninja -C ../../build/clang-Release rng_distribution
time ninja -C ../../build/clang-Release rng_distribution-check > rng_distribution.txt

./rng_distribution.pl
