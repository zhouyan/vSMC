[CMake]: http://www.cmake.org/
[Eigen]: http://eigen.tuxfamily.org/index.php
[Random123]: http://www.thesalmons.org/john/random123/releases/latest/docs/index.html
[Boost]: http://www.boost.org/
[MKL]: http://software.intel.com/en-us/articles/intel-mkl/
[TBB]: http://threadingbuildingblocks.org/
[vDist]: https://github.com/zhouyan/vDist


# Installation

To install the library just move the `include` folder into a proper place, e.g.
`/usr/local/include` on Unix-alike systems. Alternatively, one can use
[CMake][CMake]

    cd /path_to_vSMC_source 
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    sudo make install

To make the documentations one need [Doxygen][Doxygen] 1.8.0 or later.

    make docs

To build examples,

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
need [Intel MKL][MKL], [Intel TBB][TBB] and [vDist][vDist] libraries.  They
are only optional.

# Prerequisite 

This library has only two mandatory requirements: the [Eigen][Eigen]
linear algebra library and [Random123][Random123] parallel random number
library. Both of them are very portable.

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

# Tested compilers

The library itself only use standard C++98 features and is fairly portable.
For compiler support of [Eigen][Eigen] and [Random123][Random123] see their
pages respectively. In C++11 mode, the usability of `<functional>` and
`<random>` headers distributed with various implementations differs
significantly, especially `<random>`. However [Boost][Boost] can be used as a
replacement and which is known for portability. The following summaries tested
compilers. Others might work as well. [Boost][Boost] 1.4.9 is used.

## Linux

- Red Hat Enterprise Linux 6.2
  * Clang 2.8, -std=c++98 (Using [Boost][Boost])
  * GCC 4.4.6, -std=c++98 (Using [Boost][Boost])
  * GCC 4.4.6, -std=c++0x (Using [Boost][Boost], C++11 `std::function` also works, C++11 `<random>` broke, `std::uniform_real_distribution` mising)
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

- Lion 10.7.3
  * Clang 3.1, -std=c++98 -stdlib=libc++ (Using [Boost][Boost])
  * Clang 3.1, -std=c++11 -stdlib=libc++ (Using [Boost][Boost], C++11 `std::function` also works, C++11 `<random>` broke, not standard comforming)
  * Clang 3.1, -std=c++98 -stdlib=libstc++(Using [Boost][Boost])
  * Clang 3.1, -std=c++11 -stdlib=libstc++(Using [Boost][Boost])
  * Intel icpc 12.1.3, -std=c++98 (Using [Boost][Boost])
  * Intel icpc 12.1.3, -std=c++0x (Using [Boost][Boost])
