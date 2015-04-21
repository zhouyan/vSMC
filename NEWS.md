#Changes since v1 .2.0

## Important changes

The library is now C++11 only.

* Enabled C++11 features (based on [LLVM coding standard][LLVM CS]:
  - Rvalue references: [N2118][N2118]
      * But not rvalue references for `*this` or member qualifier
  - Static assert: [N1720][N1720]
  - `auto` type deduction: [N1984][N1984], [N1737][N1737]
  - Trailing return types: [N2541][N2541]
  - Lambdas: [N2927][N2927]
      * But not lambdas with default arguments
  - decltype: [N2343][N2343]
  - Nested closing right angle brackets: [N1757][N1757]
  - `nullptr`: [N2431][N2431]
  - Local and unnamed types as template arguments: [N2657][N2657]
  - Range-based for-loop: [N2930][N2930]
      * But `{}` are required around inner `do {} while ()` loops.As a result,
        `{}` are required around function-like macros inside range-based for
        loops.
  - `override` and `final`: [N2928][N2928], [N3206][N3206], [N3272][N3272]
  - Atomic operations and the C++11 memory model: [N2429][N2429]
  - Variadic templates: [N2242][N2242]
  - Explicit conversion operators: [N2437][N2437]
  - Defaulted and deleted functions: [N2346][N2346]
  - Initializer lists: [N2627][N2627]
  - Delegating constructors: [N1986][N1986]

Most changes are internal. The API remain unchanged (in C++11 mode with
supported compilers). The following are backward compatibility breaking
changes.

* `Array` is no more and its use has been replaced by `std::array`.
* `Progress` is no longer a class template.

#Changes since v1 .1.1

## New features

* `Monitor` gains a new parameter `stage`. A monitor maybe evaluated in
  different stages of the initialization and iterations. See the documents of
  `Monitor` constructor.
* A new optional argument of `Sampler::monitor` that allows setting the above
  `stage` parameter when adding a new `Monitor`.
* `StateMatrix` has new overloaded `data` member function that is equivalent to
  either `row_data` (`StateMatrix<RowMajor, Dim, T>`) or `col_data`
  (`StateMatrix<ColMajor, Dim, T>`).
* The library now optionally use `<type_traits>` standard header. The
  configuration macro is `VSMC_HAS_CXX11LIB_TYPE_TRAITS`.
* Multinomial and Residual resampling algorithms now use [Intel TBB][TBB]'s
  `tbb::parallel_sort` to improve performance. This can be disabled by setting
  `VSMC_USE_TBB` to zero even if `VSMC_HAS_TBB` is non-zero.
* New [Intel TBB][TBB] based thread local version of `RngSet`
* `WeightSet` now has a set of static member functions that can be used to
  implement weights related operations.
* New classes in `rng/u01.hpp`, `U01SequenceSorted`, `U01SequenceStratified`,
  `U01SequenceSystematic`, for generating sorted uniform random variates with
  O(N) runtime cost and O(1) memory cost. These are primarily used resampling
  algorithms within the library but can find other usages.

## Changed behaviors

* `CBlas` and `vMath` functions (vExp, etc.) no longer check threshold
  configuration macros. In particular, the macro `VSMC_CLABS_THRESHOLD` and
  `VSMC_VMATH_THRESHOLD` are no longer checked.
* `Particle` no longer check `resample_copy_from_replication_type` and
  `resample_post_copy_type` for user defined resampling behaviors. It is easier
  to write customized resampling algorithms as a `move` instead of messing with
  `Particle`'s internal this way.

## Bug fixes

* Fix Residual and related resampling algorithms in situations where the new
  system has number of particles unequal to the old system.

#Changes since v1 .1.0

## Changed behaviors

* `Sampler` summary member functions, such as `summary_data` etc., now only
  deal with floating point data, such as importance sampling estimates. Integer
  data, such as acceptance counts are dealt with new member functions
  `summary_data_int` etc.
* `Sampler` summary now output raw results of ESS and acceptance counts. They
  are no longer scaled by the number of particles.

## Bug fixes

* `Sampler` now correctly clear size history during initialization

#Changes since v1 .0.0

## New features

* Support [jemalloc][jemalloc] in `utility/aligned_memory.hpp`.
* Support storing `Particle<T>` object in [HDF5][HDF5] format.
* New function `hdf5store_new` creates a new file for storing data (trunk
  any existing files with the supplied file name).

## Changed behaviors

* `Progress` by default shows iteration number.
* `Progress::stop` by default assumes that all work is done.
* `AlignedMemory` and `AlignedAllocator` by default use [jemalloc][jemalloc] if
  it is available.
* `hdf5store_list_empty` argument `append` now has a default value, `false`.

## Bug fixes

* `hdf5size` now correctly return the number of *bytes* of data and it is no
  longer a template.
* Fix a memory bug when using "initialization by iteration"
  (`Sampler::init_by_iter(true)`).

[HDF5]: http://www.hdfgroup.org/HDF5/
[TBB]: https://www.threadingbuildingblocks.org
[jemalloc]: http://www.canonware.com/jemalloc/
[LLVM CS]: http://llvm.org/docs/CodingStandards.html

[N2118]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2006/n2118.html
[N1720]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2004/n1720.html
[N1984]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2006/n1984.pdf
[N1737]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2004/n1737.pdf
[N2541]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2008/n2541.htm
[N2927]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2009/n2927.pdf
[N2343]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2007/n2343.pdf
[N1757]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2005/n1757.html
[N2431]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2007/n2431.pdf
[N2657]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2008/n2657.htm
[N2930]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2009/n2930.html
[N2928]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2009/n2928.htm
[N3206]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2010/n3206.htm
[N3272]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2011/n3272.htm
[N2429]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2007/n2429.htm
[N2242]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2007/n2242.pdf
[N2437]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2007/n2437.pdf
[N2346]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2007/n2346.htm
[N2627]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2008/n2672.htm
[N1986]: http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2006/n1986.pdf
