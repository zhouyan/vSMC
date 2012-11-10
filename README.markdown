# Introduction

vSMC library provide a framework for implementing SMC algorithms. It provides a
core module which perform resampling, etc., operations common to all SMC
algorithms and applications. In addition, it provides the bases for
implementing parallelized samplers. The SMC algorithms are highly
parallelizable, but there are many frameworks for doing this. This library
tries to hide the different parallelization mechanism behind a unified
interface, and thus increases code reuse.

# Installation


This is a header only template C++ library. To install the library just move
the contents of the `include` directory into a proper place, e.g.,
`/usr/local/include` in Unix-alike systems. Alternatively, one can use
[CMake][CMake] (2.8 or later required),

    ```sh
    cd /path_to_vSMC_source
    mkdir build
    cd build
    cmake ..
    make install
    ```

One may need `su` or `sudo` permissions to perform the last installation step.

# Documentation

To make the documentations one need [Doxygen][Doxygen] 1.8.1 or later.

    ```sh
    make docs
    ```

The documentation can also be found [here][vSMCDoc].

# Third-party dependencies and parallelization

This library has no dependences other than C++ standard libraries (C++11). Any
C++11 language features are optional.

In particular, the library use the `<functional>`, `<random>` and
`<type_traits>` headers, which are parts of the  C++11 standard libraries.
Equivalences can be found in [Boost][Boost]. By default the library will use
the [Boost][Boost] library as C++11 implementations are not mature at the time
writing. But if the C++ implementation has them correctly implemented, the
standard headers can also be used by defining suitable macros (see the [vSMC
Macros][vSMCMacro]).

Note that this library is only tested with [Boost][Boost] 1.49 or later. Also
not all C++11 implementations of `<functional>`, `<random>` and `<type_traits>`
work properly even they are present.

The library can optionally use the [Random123][Random123] library for
parallelized random number generator, and the default behavior is assuming this
is available. The library distribute a modified version [Random123][Random123]
in the `third-party` directory. The only modification is to make it work with
[libc++][libc++].

The library support various back-ends for multi-thread parallelization, unified
under a uniform interface. One is C++11 `<thread>` (or [Boost][Boost] Thread
library as a replacement). For a full C++11 implementation, this means no
third-party dependency is required to write parallel a SMC sampler. Other
third-party parallelization include, [Intel Cilk Plus][Intel Cilk Plus], [Intel
TBB][Intel TBB] and [OpenMP][OpenMP]. In addition, this library also support
using [OpenCL][OpenCL] for GPGPU computing, though the interface is different
than others. To enable any of these, one need to define specific macros,
`VSMC_USE_STD`, `VSMC_USE_CILK`, `VSMC_USE_TBB`, `VSMC_USE_OMP`, and
`VSMC_USE_CL`, respectively.

# License

The vSMC library is distributed with a Boost license which can be found in the
`LICENSE` file distributed with the source.

[Boost]: http://www.boost.org/
[CMake]: http://www.cmake.org/
[Doxygen]: http://www.stack.nl/~dimitri/doxygen/manual.html
[Intel Cilk Plus]: http://en.wikipedia.org/wiki/Intel_Cilk_Plus
[Intel TBB]: http://threadingbuildingblocks.org/
[OpenCL]: http://www.khronos.org/opencl/
[OpenMP]: http://www.openmp.org/
[Random123]: http://www.thesalmons.org/john/random123/releases/latest/docs/index.html
[libc++]: http://libcxx.llvm.org
[vSMCDoc]: http://zhouyan.github.com/vSMC/doc/html/index.html
[vSMCMacro]: https://github.com/zhouyan/vSMC/wiki/Macros
