#!/bin/bash

ninja -C ../../build/clang-Release rng_distribution-check > rng_distribution.txt
./rng_distribution.pl
