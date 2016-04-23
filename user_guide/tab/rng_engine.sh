#!/bin/bash

echo "Running clang..."
ninja -C ../../build/clang-Release rng_engine
time ninja -C ../../build/clang-Release rng_engine-check > rng_engine_clang.txt

echo "Running gcc..."
ninja -C ../../build/gcc-Release rng_engine
time ninja -C ../../build/gcc-Release rng_engine-check > rng_engine_gcc.txt

echo "Running intel..."
ninja -C ../../build/intel-Release rng_engine
time ninja -C ../../build/intel-Release rng_engine-check > rng_engine_intel.txt
