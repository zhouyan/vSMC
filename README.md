# Introduction

The vSMC library provides a framework for implementing Monte Carlo algorithms.
It was originally developed for the purpose of parallel implementation of SMC
algorithms. But it now supports other algorithms as well. The library provides
basic building blocks for implementing parallelizable Monte Carlo algorithms.

# Installation

This is a header only template C++ library. To install the library just move
the contents of the `include` directory into a proper place, e.g.,
`/usr/local/include` on Unix-alike systems.

# Documentation

[Doxygen][doxygen] generated reference manuals for the
[master][vSMCDoxygenMaster] and [develop][vSMCDoxygenDevelop] branches, as well
as for individual releases can be found online (see [release
notes][vSMCReleases] for links). A [Manual][VSMCManual] is also provided.

# Third-party dependencies

This library requires a working BLAS implementation. Some of the library's
functionalities can only be used if a optional dependencies are present.
Notably, [HDF5][hdf5], [TBB][tbb], [OpenMP][omp] and [MKL][mkl]. One can tell
the library that these optional features are available by defining
configuration macros such as `-DVSMC_HAS_HDF5=1` during compilation.

# Compiler support

This library makes heavy use of some template metaprogramming techniques. It
requires a C++11 standard conforming compiler.

This library has been regularly tested with recent [Clang][clang], [GCC][gcc]
and [Intel C++ Compiler][icpc] in C++11 mode.

Other compilers might work but are not tested. Complete C++11 implementation is
required with thread-local storage as an exception.

# Examples

Examples are in the `example` subdirectory, to build them,
~~~sh
export CXXFLAGS="-std=c++11"
cd /path_to_vSMC_source
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make example
~~~
Some examples may only be built if optional dependencies are present.

# License

The vSMC library is distributed with a 2-clause BSD license which can be found
in the `LICENSE` file distributed with the source.

[clang]: http://clang.llvm.org
[cmake]: http://www.cmake.org
[doxygen]: http://www.stack.nl/~dimitri/doxygen
[gcc]: http://gcc.gnu.org
[hdf5]: http://www.hdfgroup.org
[icpc]: http://software.intel.com/en-us/intel-compilers
[mkl]: https://software.intel.com/en-us/intel-mkl
[omp]: http://www.openmp.org
[tbb]: http://threadingbuildingblocks.org
[vSMCDoxygenDevelop]: http://zhouyan.github.io/vSMCDoc/develop
[vSMCDoxygenMaster]: http://zhouyan.github.io/vSMCDoc/master
[vSMCManualDevelop]: http://zhouyan.github.io/vSMCDoc/develop/manual.pdf
[vSMCManualMaster]: http://zhouyan.github.io/vSMCDoc/master/manual.pdf
[vSMCReleases]: https://github.com/zhouyan/vSMC/releases
