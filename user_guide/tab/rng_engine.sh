#!/bin/bash

ninja -C ../../build/clang-Release rng_engine-check > rng_engine_clang.txt
ninja -C ../../build/gcc-Release rng_engine-check > rng_engine_gcc.txt
ninja -C ../../build/intel-Release rng_engine-check > rng_engine_intel.txt
