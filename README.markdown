[CMake]: http://www.cmake.org/
[Eigen]: http://eigen.tuxfamily.org/index.php
[Random123]: http://www.thesalmons.org/john/random123/releases/latest/docs/index.html
[Boost]: http://www.boost.org/
[MKL]: http://software.intel.com/en-us/articles/intel-mkl/
[TBB]: http://threadingbuildingblocks.org/
[vDist]: https://github.com/zhouyan/vDist


# Installation

To install the library just move the `include` folder into a proper place, e.g.
`/usr/local/include` on Unix-alike systems.

# Prerequisite 

This library has only two mandatory requirement: the [Eigen](Eigen)
linear algebra library and [Random123](Random123) parallel random number
library. Both of them are very portable.

In addition, the library use the `<functional>` and `<random>` library, which
can be found in C++11 or [Boost](Boost). By default the library will use the
Boost library. But if the C++ implementation has these C++11 features
correctly implemented, they can also be used.

Note that this library is only tested with [Boost](Boost) 1.49 or later.
Earlier versions of [Boost](Boost) may not work, mainly due the possible bugs
in the Random library. Also not all C++11 (or C++0x) implementations of
`<random>` work properly. At the time of writing, `libc++` distributed with
Xcode 4.3.2 is one example of failure.

More specifically, the library will first look for the macro
`V_SMC_USE_STD_FUNCTION`. If it is defined, `std::function` will be used.
Otherwise the library will use [Boost](Boost), in particular
`boost::function`. The same procedure is followed for the `<random>` library.
One can put appropriate macros in the `vSMC/internal/config.hpp` header or use
compiler flags.

# Examples

To build the examples, one need the [CMake](Cmake) program.

To build all the examples, one may also need [Intel MKL](MKL), [Intel
TBB](TBB) and [vDist](vDist) libraries. They are only optional.
