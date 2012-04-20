# Installation

To install the library just move the `include` folder into a proper place,
e.g. `/usr/local/include` on Unix-alike systems. Alternatively, one can use
[CMake][CMake] (2.6.4 or later required).

    cd /path_to_vSMC_source 
    mkdir build
    cd build
    cmake ..
    sudo make install

# Documentation

To make the documentations one need [Doxygen][Doxygen] 1.8.0 or later.

    make docs

# Testing

## Prerequisite 

This library has only two mandatory requirements: the [Eigen][Eigen] linear
algebra library and [Random123][Random123] parallel random number library.
Both of them are very portable.

In addition, the library use the `<functional>` and `<random>` library, which
can be found in C++11 or [Boost][Boost]. By default the library will use the
Boost library. But if the C++ implementation has these C++11 features
correctly implemented, they can also be used.

Note that this library is only tested with [Boost][Boost] 1.49 or later.
Earlier versions of [Boost][Boost] may not work, mainly due the possible bugs
in the Random library. Also not all C++11 (or C++0x) implementations of
`<random>` work properly. At the time of writing, `libc++` distributed with
Xcode 4.3.2 is one example of failure.

More specifically, the library will first look for the macro
`V_SMC_USE_STD_FUNCTION`. If it is defined, `std::function` will be used.
Otherwise the library will use [Boost][Boost], in particular
`boost::function`. The same procedure is followed for the `<random>` library.
One can put appropriate macros in the `vSMC/internal/config.hpp` header or use
compiler flags.

## Building and testing

To build test examples,

    make buildtests

To run the tests,

    make test

Or

    ctest

To build and run tests in a single step

    make check

Note that [CMake][CMake] generated `Makefile` does not build test executables
before run `ctest`, so you need either run `make build tests` before `make
test` or `ctest`, or run `make check`. To build all the examples, one may also
need [Intel MKL][MKL], [Intel TBB][TBB] and [vDist][vDist] libraries. They are
only optional.

Also the Particle filter and Gaussian mixture model may take some non-trivial
run-time, therefore, use `-DCMAKE_BUILD_TYPE=Rlease` to build optimized
binary.  To run them, invoke `ctest -C Release` or `make check` to run the
tests. Without the configuration these examples won't be run.

## Finding libraries

Without any configuration, the distributed `cmake` script may not be able to
find all the libraries.

Under Unix-like system (Linux and Mac OS X) For [Boost][Boost], [Eigen][Eigen],
[Random123][Random123], the following directories are searched,
`/usr/local/include`, `/usr/local/lib`, `/usr/local/lib64`, and system
directories and `INCLUDE` and `LIBRARY_PATH` environment variables For [Intel
MKL][Intel MKL], additional directories searched are `/opt/intel/mkl/lib`,
`/opt/intel/mkl/lib/ia32`, `/opt/intel/mkl/intel64` and corresponding `lib`
directories under `MKLROOT` environment variables. For [Intel TBB][Intel TBB]
similar additional directories as for [Intel MKL][Intel MKL] are searched
(replace `mkl` by `tbb`)

In addition, on Windows when [Intel TBB][Intel TBB] or [Intel MKL][Intel MKL]
are used, the DLL files also need to be found. However, currently examples
that use [Intel MKL][Intel MKL] will not be built on Windows with the CMake
scripts distributed with this library. However it can still be used in users'
own project if one know how to link it.

If the script cannot find the libraries, try set one or more of `BOOST_ROOT`,
`Eigen_INC_PATH`, `Random123_INC_PATH`, `MKL_INC_PATH`, `MKL_LIB_PATH`,
`TBB_INC_PATH`, `TBB_LIB_PATH` and `TBB_DLL_PATH` when invoking `cmake`. For
example

    cmake .. -DCMAKE_BUILD_TYPE=Release -DBOOST_ROOT=/opt/boost/include

Default values are provided for these macros under Windows if they are not set
by the user, assuming one has installed these libraries in these directories,

- `BOOST_ROOT`: `C:/Program Files/Boost`
- `Eigen_INC_PATH`: `C:/Program Files/Eigen`
- `Random123_INC_PATH`: `C:/Program Files/Random123/include`
- `TBB_INC_PATH`: `C:/Program Files/TBB/include`
- `TBB_LIB_PATH`: `C:/Program Files/TBB/lib/intel64/vc10`
- `TBB_DLL_PATH`: `C:/Program Files/TBB/bin/intel64/vc10`

The last two will change depending on the system and compiler, for example
with MSVC 2008 on a 32-bit system `TBB_LIB_PATH` will be
 `C:/Program Files/TBB/lib/ia32/vc9`. However so far this library only works
with MSVC 2010, due to problems with [Random123][Random123]. (Well, one can
only blame Microsoft for not implement `stdint.h` in MSVC 2008, even after
almost 10 years of C99 released.)

# Tested compilers

The library itself only use standard C++98 features and is fairly portable.
For compiler support of [Eigen][Eigen] and [Random123][Random123] see their
pages respectively. In C++11 mode, the usability of `<functional>` and
`<random>` headers distributed with various implementations differs
significantly, especially `<random>`. However [Boost][Boost] can be used as a
replacement and which is known for portability. The following summaries tested
compilers. [Boost][Boost] 1.4.9 is used. Others might work as well. The aim is
that vSMC shall work anywhere [Eigen][Eigen], [Random123][Random123] and
[Boost][Boost] works.

## Linux

- Red Hat Enterprise Linux 6.2
  * Clang 2.8, -std=c++98 (Using [Boost][Boost])
  * GCC 4.4.6, -std=c++98 (Using [Boost][Boost])
  * GCC 4.4.6, -std=c++0x (Using [Boost][Boost], C++11 `std::function` also
    works, C++11 `<random>` broke, `std::uniform_real_distribution` mising)
  * Intel icpc 12.1.3, -std=c++98 (Using [Boost][Boost])
  * Intel icpc 12.1.3, -std=c++0x (Using [Boost][Boost], C++11 `std::function` also works, C++11 `<random>` broke, `std::uniform_real_distribution` mising)

- Ubuntu 12.04
  * Clang 3.1, -std=c++98 (Using [Boost][Boost])
  * Clang 3.1, -std=c++0x (Using [Boost][Boost] or C++11 headers)
  * GCC 4.6.3, -std=c++98 (Using [Boost][Boost])
  * GCC 4.6.3, -std=c++0x (Using [Boost][Boost] or C++11 headers)
  * Intel icpc 12.1.3, -std=c++98 (Using [Boost][Boost])
  * Intel icpc 12.1.3, -std=c++0x (Using [Boost][Boost])

Note that `icpc` use `libstdc++` distributed with the system. 

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

Earlier versions of MSVC does not work, this is mainly a dependency problem of
[Random123][Random123].

[Boost]: http://www.boost.org/
[CMake]: http://www.cmake.org/
[Doxygen]: http://www.stack.nl/~dimitri/doxygen/manual.html
[Eigen]: http://eigen.tuxfamily.org/index.php
[Intel MKL]: http://software.intel.com/en-us/articles/intel-mkl/
[Random123]: http://www.thesalmons.org/john/random123/releases/latest/docs/index.html
[SMCTC]: http://www2.warwick.ac.uk/fac/sci/statistics/staff/academic-research/johansen/smctc/
[Intel TBB]: http://threadingbuildingblocks.org/
[vDist]: https://github.com/zhouyan/vDist/
