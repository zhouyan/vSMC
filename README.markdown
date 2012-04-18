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
can be found in C++11, TR1, or [Boost](Boost). By default the library will use
the Boost library. But if the C++ implementation has these C++11 or TR1
features, they can also be used.

More specifically, the library will first look for the macro
`V_SMC_USE_STD_FUNCTION`. If it is defined, `std::function` will be used.
Otherwise the library will look for the `V_SMC_USE_TR1_FUNCTION`. If it is
defined, then `std::tr1::function` will be used. If both test fail, the
function will use [Boost](Boost) and use `boost::function`. The same procedure
is followed for the `<random>` library. One can put appropriate macros in the
`vSMC/internal/config.hpp` header or use compiler flags.

# Examples

To build the examples, one need the [CMake](Cmake) program.

To build all the examples, one may also need [Intel MKL](MKL), [Intel
TBB](TBB) and [vDist](vDist) libraries. They are only optional.
