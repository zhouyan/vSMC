# Introduction

The vSMC library provides a framework for implementing SMC algorithms. It has a
core module which performs resampling, etc., operations common to all SMC
algorithms and applications. In addition, it provides the bases for
implementing parallelized samplers. The SMC algorithms are highly
parallelizable, but there are many frameworks for doing this. This library
tries to hide the different parallelization mechanism behind a unified
interface, and thus increases code reuse.

# Installation

This is a header only template C++ library. To install the library just move
the contents of the `include` directory into a proper place, e.g.,
`/usr/local/include` in Unix-alike systems. Alternatively, one can use
[CMake][CMake] (2.8.3 or later required),
~~~sh
cd /path_to_vSMC_source
mkdir build
cd build
cmake ..
make install
~~~
One may need `su` or `sudo` permissions to perform the last installation step.

# Documentation

To make the documentations one need [Doxygen][Doxygen] 1.8.3 or later.
~~~sh
make docs
~~~
The documentation of the [master][vSMCDocMaster] and [develop][vSMCDocDevelop]
branches can be found online.

# Examples

Examples are in the `example` subdirectory, to build them,
~~~sh
cd /path_to_vSMC_source
mkdir build
cd build
cmake ..
make example
~~~
Most examples also come with their own `README` files that give relevant
references.

# Parallelization backends

The library support various backends for multi-thread parallelization, unified
under a uniform interface. The primary backends are [OpenMP][OpenMP] and [Intel
TBB][Intel TBB]. Other backends are available in the [vSMC Extra][vSMC Extra]
repository, including MPI and [OpenCL][OpenCL].

# Third-party dependencies

This library has no dependences other than C++ standard libraries (C++11). Any
C++11 language features are optional.

In particular, the library use the `<functional>` and `<random>` headers, which
are parts of the C++11 standard libraries. Equivalences can be found in recent
versions of [Boost][Boost]. The library does its best to detect a usable C++11
solution and falls back to [Boost][Boost] if it fails to do so. This behavior
can be changed explicitly through configuration macros.

# Compiler support

This library makes heavy use of some template metaprogramming techniques. It
requires a C++11 standard conforming compiler.

This library has been regularly tested with recent [Clang][Clang], [GCC][GCC]
and [Intel C++ Compiler][icpc] in C++11 mode.

[Microsoft Visual C++][MSVC] 2015 or later are also supported. However, this
compiler is tested less regularly.

Other compilers might work but are not tested.

# License

The vSMC library is distributed with a 2-clause BSD license which can be found
in the `LICENSE` file distributed with the source.

[Apple GCD]: http://en.wikipedia.org/wiki/Grand_Central_Dispatch
[Boost]: http://www.boost.org
[CMake]: http://www.cmake.org
[Clang]: http://clang.llvm.org
[Doxygen]: http://www.stack.nl/~dimitri/doxygen/manual.html
[GCC]: http://gcc.gnu.org
[Intel Cilk Plus]: https://www.cilkplus.org
[Intel TBB]: http://threadingbuildingblocks.org
[MS PPL]: http://msdn.microsoft.com/en-us/library/dd492418.aspx
[MSVC]: http://msdn.microsoft.com/en-us/vstudio//default.aspx
[OpenCL]: http://www.khronos.org/opencl
[OpenMP]: http://www.openmp.org
[icpc]: http://software.intel.com/en-us/intel-compilers
[vSMCDocMaster]: http://zhouyan.github.io/vSMCDoc/master
[vSMCDocDevelop]: http://zhouyan.github.io/vSMCDoc/develop
[vSMC Extra]: https://github.com/zhouyan/vSMCExtra.git
