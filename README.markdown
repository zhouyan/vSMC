# Installation

To install the library just move the `include` folder into a proper place,
e.g. `/usr/local/include` in Unix-alike systems. Alternatively, one can use
[CMake][CMake] (2.8 or later required),

    cd /path_to_vSMC_source
    mkdir build
    cd build
    cmake ..
    sudo make install

# Documentation

To make the documentations one need [Doxygen][Doxygen] 1.8.1 or later.

    make docs

The documentation can also be found [here][vSMCDoc].

# Testing

## Prerequisite

This library has no dependences other than C++11 standard libraries.

In particular, the library use the `<functional>`, `<random>` and `<type_traits>`
headers, which are parts of the  C++11 standard libraries or can be found in
[Boost][Boost]. By default the library will use the [Boost][Boost] library. But
if the C++ implementation has them correctly implemented, the standard headers
can also be used.

Note that this library is only tested with [Boost][Boost] 1.49 or later. Also
not all C++11 (or C++0x) implementations of `<functional>`, `<random>` and
`<type_traits>` work properly.

Any C++11 language features are optional.

## Building and testing

To build test examples,

    make buildtests

To run the tests,

    ctest -C Release

Note that [CMake][CMake] generated `Makefile` does not build test executables
before run `ctest` in the `test` target, so you need run `make buildtests`
manually first

`make buildtests` will also build the examples, one is a simple particle
filter, in `test/pf`, the others are a Gaussian mixture model, non-linear
ordinary differential equations models and Positron Emission Tomography
compartmental model with SMC, in `test/gmm` and `test/pet` respectively. The
examples may take some non-trivial run-time, therefore they are not run when
invoking `ctest`.

Without [CMake][CMake] one can go to the `test` directory and build the
examples manually. For example,

    cd test/pf
    g++ -O3 \
      -I /path_to_v_smc_headers \
      -I /path_to_eigen_headers \
      -I /path_to_random123_headers \
      -I /path_to_tbb_headers -L /path_to_tbb_libraries -ltbb \
      -o pf_tbb pf_tbb.cpp

For every example, they can be possibly built with [OpenMP][OpenMP], [Intel
TBB][Intel TBB] support in addition to sequential implementations. These will
require [OpenMP][OpenMP] compiler support and the [Intel TBB][Intel TBB]
library support respectively. The particle filter example can also be built
[OpenCL][OpenCL] for a GPU implementation. All examples other than the particle
filter requires the [Eigen][Eigen] library and [Boost][Boost] Program Options
library. They may also require some additional C99 math functions. When the
[Eigen][Eigen] library is available, a vectorized implementation of the
particle filter can also be built. Last, if C++11 `<chrono>` or corresponding
[Boost][Boost] library is available, then the particle filter will also be
timed.

# License

The vSMC library is distributed with a two-clauses BSD license which can be
found in the COPYING file distributed with the source.

[Boost]: http://www.boost.org/
[CMake]: http://www.cmake.org/
[Doxygen]: http://www.stack.nl/~dimitri/doxygen/manual.html
[Eigen]: http://eigen.tuxfamily.org/index.php
[Random123]: http://www.thesalmons.org/john/random123/releases/latest/docs/index.html
[SMCTC]: http://www2.warwick.ac.uk/fac/sci/statistics/staff/academic-research/johansen/smctc/
[Intel TBB]: http://threadingbuildingblocks.org/
[OpenCL]: http://www.khronos.org/opencl/
[OpenMP]: http://www.openmp.org/
[vSMCDoc]: http://zhouyan.github.com/vSMC/doc/html/index.html
