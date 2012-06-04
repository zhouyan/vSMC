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

# Testing

## Prerequisite

This library has only two mandatory requirements: the [Eigen][Eigen] linear
algebra library and [Random123][Random123] parallel random number library.
Both of them are very portable.

In addition, the library use the `<functional>` and `<random>` libraries, which
are C++11 standard libraries or can be found in [Boost][Boost]. By default the
library will use the [Boost][Boost] library. But if the C++ implementation has
them correctly implemented, the standard headers can also be used.

Note that this library is only tested with [Boost][Boost] 1.49 or later. Also
not all C++11 (or C++0x) implementations of `<functional`> and `<random>` work
properly.

The library will first look for the macro `V_SMC_USE_STD_FUNCTION`. If it is
defined, `std::function` will be used. Otherwise the library will use
[Boost][Boost], in particular `boost::function`. The same procedure is followed
for the `<random>` library. One can put appropriate macros in the
`vSMC/internal/config.hpp` header or use compiler flags.

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
    g++ -O3 \
      -I /path_to_v_smc_headers \
      -I /path_to_eigen_headers \
      -I /path_to_random123_headers \
      -I /path_to_tbb_headers -L /path_to_tbb_libraries \
      -o pf_tbb pf_tbb.cpp

## Additional libraries used by examples

There are three versions of the particle filter example, **pf_seq**,
**pf_tbb**, and **pf_vec**. To build **pf_seq** and **pf_vec** one only need
the required libraries, namely [Eigen][Eigen], [Random123][Random123] and
suitable `<functional>`, `<random>` or [Boost][Boost]. To build **pf_tbb** one
also need the [Intel TBB][Intel TBB] library.

The Gaussian mixture model example can be built with or without the [Intel
TBB][Intel TBB] library. If the library is found, by default it will be built
with it. One can define `-DGMM_SEQUENTIAL=ON` when invoking `cmake` to disable
the parallelization. If one also define `-DGMM_STATIC=ON` then the examples
will be built without [Intel TBB][Intel TBB] and linked statically. Similar
options are available for the Positron Emission Tomography compartmental model
example, just replace `GMM` with `PET`.

For how to find these libraries see files in `cmake/` and see [CMake][CMake]'s
documentation for general usage of this tool.

There is another example that will not be built by the `cmake` system,
**pf_cl**, which use OpenCL for parallelization. This is due to the fact that
finding OpenCL runtime is a quite non-portable thing and we don't have the time
to deal with all those different OpenCL implementations for now. However, the
OpenCL backend is intended to be usable by any OpenCL standard conforming
implementation. To build this example on a Mac, one can ues

    cd /test/pf
    clang++ -O3 \
      -I /path_to_v_smc_headers \
      -I /path_to_eigen_headers \
      -I /path_to_random123_headers \
      -framework OpenCL
      -o pf_cl pf_cl.cpp

The compiler flags may need to be changed for different OpenCL implementations.
Also you may need to change the compiler flags for OpenCL in the source file,
`pf.hpp`, search for `FIXME`.

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

## Linux

- With [Boost][Boost]
  * GCC 4.4, 4.5, 4.6,
  * Intel icpc 12.0, 12.1
    tested against GCC 4.4, 4.5, 4.6
  * Clang 2.8, 2.9, 3.0, 3.1, SVN

The above compilers are also tested with flag `-std=c++98`, `-std=gnu98`, etc.
and `-std=c++0x`, `-std=c++11` etc. Note that all compilers above use
`libstdc++` as standard libraries. With `libstdc++` distributed by GCC 4.4 or
later, and appropriate compiler flags, using C++11 `<functional>` instead of
[Boost][Boost] also works. With `libstdc++` distributed by GCC 4.5 or later,
C++11 `<random>` also works. Note that Intel icpc disable C++0x mode for
GCC 4.6 system.

## Mac OS X

- Lion 10.7.3, Xcode 4.3.2
  * Clang 3.1, -std=c++98 -stdlib=libc++ (Using [Boost][Boost])
  * Clang 3.1, -std=c++11 -stdlib=libc++ (Using [Boost][Boost], C++11
    `std::function` also works, C++11 `<random>` broke, not standard
    comforming)
  * Clang 3.1, -std=c++98 -stdlib=libstc++(Using [Boost][Boost])
  * Clang 3.1, -std=c++11 -stdlib=libstc++(Using [Boost][Boost])
  * Intel icpc 12.1.3, -std=c++98 (Using [Boost][Boost])
  * Intel icpc 12.1.3, -std=c++0x (Using [Boost][Boost])

## Windows

- Windows 7
  * MSVC 2010 (Version 10) (Using [Boost][Boost] or C++11 headers)

Note that MSVC cannot compile all the examples.

Earlier versions of MSVC does not work, this is mainly a dependency problem of
[Random123][Random123].

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
