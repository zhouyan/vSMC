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
The documentation can also be found [here][vSMCDoc]. A
[tutorial][vSMCTutorial] is also available. However, it describes an earlier
version of the library.  There are a few incompatibilities with the current
version. It is still highly relevant. Users shall use the Doxygen generated
documentations when things do not work exactly the same way as in the tutorial.

# Examples

[Examples][vSMCExample] are now hosted separately. To get and build them,
~~~sh
cd /path_to_vSMC_source
git clone https://github.com/zhouyan/vSMCExample.git
mkdir build
cmake ..
make example
~~~
Most examples also come with their own `README` files that give relevant
references.

# Parallelization backends

The library support various backends for multi-thread parallelization, unified
under a uniform interface. One is C++11 concurrency. For a full C++11
implementation, this means no third-party dependency is required to write a
parallel SMC sampler. Other third-party parallelization include, [Intel Cilk
Plus][Intel Cilk Plus], [Intel TBB][Intel TBB] and [OpenMP][OpenMP]. [Apple
Grand Central Dispatch][Apple GCD] is also supported on Mac OS X and on Linux
via [libdispatch][libdispatch]. [Microsoft Parallel Patterns Library][MS PPL]
is supported on Windows when compiled with MSVC 2010 or later. In addition,
this library also support using [OpenCL][OpenCL] for GPGPU computing, though
the interface is different than others.

# Third-party dependencies

This library has no dependences other than C++ standard libraries (C++11). Any
C++11 language features are optional.

In particular, the library use the `<functional>` and `<random>` headers, which
are parts of the C++11 standard libraries. Equivalences can be found in
[Boost][Boost]. By default the library will use the [Boost][Boost] library as
C++11 implementations are not mature at the time writing. But if the C++
implementation has them correctly implemented, the standard headers can also be
used by defining suitable macros (see reference manual for details).

Note that this library is only tested with [Boost][Boost] 1.49 or later. Also
not all C++11 implementations of `<functional>` and `<random>` work properly
even they are present.

# Compiler support

This library makes heavy use of some template metaprogramming techniques. It
requires a standard conforming compiler. Fortunately, most commonly used
modern compilers, at least in C++98 mode, is able to compile the examples
distributed with the library.

This library has been regularly tested with recent [Clang][Clang], [GCC][GCC]
and [Intel C++ Compiler][icpc], in both C++98 and C++11 modes. In particular,
[Clang][Clang] 3.3 and later with [libc++][libc++] and [GCC][GCC] 4.7 and later
support all the C++11 features used by the library very well. [Intel C++
Compiler][icpc] when used with [GCC][GCC] 4.7's standard library can also
support all the C++11 features. When it is used with [GCC][GCC] 4.8's standard
library, though all features are supported, some examples fail to compile when
complex template constructs are involved. The issues are still under
investigation. The current workaround is to use the [Boost][Boost] Function
library instead of the standard library `<functional>` (by defining the flag
`-DVSMC_HAS_CXX11LIB_FUNCTIONAL=0`) when using this compiler configuration in
C++11 mode.

[Microsoft Visual C++][MSVC] is also supported. Version 2008 and later are able
to compile the examples in C++98 mode. Version 2012 and later support most of
the C++11 features. However, this compiler is tested less regularly.

Other compilers such as [Open64][Open64] were previously tested in C++98 mode
(most of them don't support C++11 at all). Future developments will rely more
on C++11 features. There are likely to be new (optional) features that are
C++11 only. Therefore, these outdated compilers won't be tested anymore.
However, for the foreseeable future, all basic features should be supported by
a C++98 compiler.

# License

The vSMC library is distributed with a 2-clause BSD license which can be found
in the `LICENSE` file distributed with the source.

[Apple GCD]: http://en.wikipedia.org/wiki/Grand_Central_Dispatch
[Boost]: http://www.boost.org/
[CMake]: http://www.cmake.org/
[Clang]: http://clang.llvm.org
[Doxygen]: http://www.stack.nl/~dimitri/doxygen/manual.html
[GCC]: http://gcc.gnu.org
[Intel Cilk Plus]: https://www.cilkplus.org
[Intel TBB]: http://threadingbuildingblocks.org/
[MS PPL]: http://msdn.microsoft.com/en-us/library/dd492418.aspx
[MSVC]: http://msdn.microsoft.com/en-us/vstudio//default.aspx
[Open64]: http://www.open64.net
[OpenCL]: http://www.khronos.org/opencl/
[OpenMP]: http://www.openmp.org/
[icpc]: http://software.intel.com/en-us/intel-compilers
[libc++]: http://libcxx.llvm.org
[libdispatch]: http://libdispatch.macosforge.org/
[vSMCDoc]: http://zhouyan.github.io/vSMCDoc/doc/html
[vSMCExample]: https://github.com/zhouyan/vSMCExample
[vSMCTutorial]: http://arxiv.org/pdf/1306.5583v1.pdf
