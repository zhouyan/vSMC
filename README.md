# Introduction

The vSMC library provides a framework for implementing SMC algorithms. It has a
modules that perform resampling, etc., operations common to all SMC algorithms
and applications. In addition, it provides bases for implementing parallelized
samplers. The SMC algorithms are highly parallelizable, but there are many
frameworks for doing this. This library tries to hide the different
parallelization mechanism behind a unified interface, and thus increases code
reuse.

# Installation

This is a header only template C++ library. To install the library just move
the contents of the `include` directory into a proper place, e.g.,
`/usr/local/include` on Unix-alike systems. Alternatively, one can use
[CMake][CMake] (3.0.0 or later required),
~~~sh
cd /path_to_vSMC_source
mkdir build
cd build
cmake ..
make install
~~~
One may need administrator permissions to perform the last installation step or
alternatively one can define the [CMake][CMake] variable `CMAKE_INSTALL_PREFIX`
to change the destination of installation.

# Documentation

The documentation of the [master][vSMCDocMaster] and [develop][vSMCDocDevelop]
branches, as well as for individual releases can be found online. A [User
guide][VSMCUserGuide] is also provided for the develop branch.

# Third-party dependencies

This library requires a working BLAS/LAPACK implementation, with the standard C
interface headers (`cblas.h` and `lapacke.h`). Some of the library's
functionalities can only be used if a optional dependencies are present.
Notably, [HDF5][HDF5], [TBB][TBB], [OpenMP][OpenMP] and [MKL][MKL]. One can
tell the library that these optional features are available by defining
configuration macros such as `-DVSMC_HAS_HDF5=1` during compilation.

# Parallelization backends

This library support various backends for multi-thread parallelization, unified
under a uniform interface. The primary backends are [OpenMP][OpenMP] and
[TBB][TBB].

# Compiler support

This library makes heavy use of some template metaprogramming techniques. It
requires a C++11 standard conforming compiler.

This library has been regularly tested with recent [Clang][Clang], [GCC][GCC]
and [Intel C++ Compiler][icpc] in C++11 mode.

Other compilers might work but are not tested. Complete C++11 implementation is
required.

# Examples

Examples are in the `example` subdirectory, to build them,
~~~sh
cd /path_to_vSMC_source
mkdir build
cd build
cmake ..
make example
~~~
Some examples may only be built if some optional dependencies are present.

# License

The vSMC library is distributed with a 2-clause BSD license which can be found
in the `LICENSE` file distributed with the source.

[CMake]: http://www.cmake.org
[Clang]: http://clang.llvm.org
[Doxygen]: http://www.stack.nl/~dimitri/doxygen/manual.html
[GCC]: http://gcc.gnu.org
[HDF5]: http://www.hdfgroup.org
[MKL]: https://software.intel.com/en-us/intel-mkl
[TBB]: http://threadingbuildingblocks.org
[OpenCL]: http://www.khronos.org/opencl
[OpenMP]: http://www.openmp.org
[icpc]: http://software.intel.com/en-us/intel-compilers
[vSMCDocDevelop]: http://zhouyan.github.io/vSMCDoc/develop
[vSMCDocMaster]: http://zhouyan.github.io/vSMCDoc/master
[vSMCUserGuide]: http://zhouyan.github.io/vSMCDoc/develop/user_guide.pdf
