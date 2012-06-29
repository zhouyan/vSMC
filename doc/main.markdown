vSMC {#mainpage}
================

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

This library has two mandatory requirements other than standard libraries: the
[Eigen][Eigen] linear algebra library and [Random123][Random123] parallel
random number library. Both of them are very portable.

In addition, the library use the `<functional>`, `<random>` and `<type_traits>`
headers, which are parts of the  C++11 standard libraries or can be found in
[Boost][Boost]. By default the library will use the [Boost][Boost] library. But
if the C++ implementation has them correctly implemented, the standard headers
can also be used.

Note that this library is only tested with [Boost][Boost] 1.49 or later. Also
not all C++11 (or C++0x) implementations of `<functional`>, `<random>` and
`<type_traits>` work properly.

The library will first look for the macro `VSMC_USE_STD_FUNCTION`. If it is
defined, `std::function` will be used. Otherwise the library will use
[Boost][Boost], in particular `boost::function`. The same procedure is followed
for the `<random>` library with `VSMC_USE_STD_RANDOM` and `<type_traits>` with
`VSMC_USE_STD_TYPE_TRAITS` macros One can put appropriate macros in the
`vsmc/internal/config.hpp` header or use compiler flags.

## Building and testing

To build test examples,

    make buildtests

To run the tests,

    make test

Or

    ctest

Note that [CMake][CMake] generated `Makefile` does not build test executables
before run `ctest` in the `test` target, so you need run `make buildtests`
before `make test` or `ctest`.

`make buildtests` will also build the examples, one is a simple particle
filter, in `test/pf`, the others are a Gaussian mixture model and Positron
Emission Tomography compartmental model with SMC, in `test/gmm` and `test/pet`
respectively. The examples may take some non-trivial run-time, therefore they
are not run when invoking `make test` or `ctest`. To run the example, one can
invoke `ctest -C Release`. Alternatively, one can use

    make check

to build all tests and examples, and run all of them.

Without [CMake][CMake] one can go to the `test` directory and build the
examples manually. For example,

    cd test/pf
    g++ -std=c++0x -O3 \
      -I /path_to_v_smc_headers \
      -I /path_to_eigen_headers \
      -I /path_to_random123_headers \
      -I /path_to_tbb_headers -L /path_to_tbb_libraries -ltbb \
      -o pf_tbb pf_tbb.cpp

There are some other more complex realistic examples and other configurations
for the simple particle filter. They may require additional optional libraries.

# Tested compilers

The library itself only use standard C++98 features and is fairly portable.
For compiler support of [Eigen][Eigen] and [Random123][Random123] see their
pages respectively. In C++11 mode, the usability of `<functional>` and
`<random>` headers distributed with various implementations differs
significantly. However [Boost][Boost] can be used as a replacement and which is
well known for portability. The following summaries tested compilers.
[Boost][Boost] 1.49 is used. Others might work as well. The aim is that vSMC
shall work anywhere [Eigen][Eigen], [Random123][Random123] and [Boost][Boost]
works.

- Linux
  * GCC 4.4, 4.5, 4.6,
  * Intel icpc 12.0, 12.1
  * Clang 2.8, 2.9, 3.0, 3.1, SVN

- Mac OS X
  * Clang 3.1
  * Intel icpc 12.1

- Windows
  * MSVC 2010

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
[vSMCDoc]: http://zhouyan.github.com/vSMC/doc/html/index.html


