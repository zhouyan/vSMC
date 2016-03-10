# Changes since v2.1.0

## Changed behaviors

* `MatrixOrder` is now an alias to `MatrixLayout`.
* CBLAS and LAPACK are now required dependency.
* Path sampling support is removed. It can be easily done through the more
  general `Monitor`. See the GMM example.

## New features

* `Particle` now has a new `sp` member that return `SingleParticle` objects.
* `NormalMVDistribution` and related functions for generating multivariate
  Normal distribution random variate. 
* New module "Random Walk" that implements generic random walk MCMC kernels and
  Normal distribution based proposals. Both scalar and multivariate cases are
  supported.
* `Covariance` is a new class that can compute the covariance matrix from
  (weighted) samples, and output the matrix in variance formats.
* `cov_chol` transform a covariance matrix, stored in various formats, into the
  lower triangular part of its Cholesky decomposition, stored as a packed row
  major matrix

## Changed behaviors

* `AlignedAllocator` template parameter `Memory` now requires its member
  functions to be static and the class stateless.
* `U01Distribution` now produce random numbers in interval (0, 1] if the
  unsigned integer results from the RNG has more bits than the significant bits
  of the floating point types. Otherwise it produce random numbers in (0, 1).
  This behavior affect all distributions. But it shall not be visible within
  user program.

## Removed features

* `U01CCDistribution`, etc., are removed. The functionally of generating fixed
  point random numbers within interval [0, 1] is preserved through `u01_lr`,
  etc., functions which convert a random unsigned integer to floating points
  numbers within desired ranges. It is extended to support all unsigned integer
  types and floating points types.

# Changes since v2.0.0

## Important changes

The mutation (MCMC) steps are now performed after the resampling of
initialization step.

# Changes since v1.2.0

## Important changes

The library is now C++11 only. GCC 4.8.1, Clang 3.4, Intel C++ 2015 all
provides full C++11 support. MSVC is the only one lagging behind. At the
moment, MSVC 2015 support is considered to be the minimum.

Most changes are internal. The API remain unchanged (in C++11 mode with
supported compilers). The following are backward compatibility breaking
changes.

* Everything in the namespace `vsmc::cxx11` is gone. Replace `vsmc::cxx11` with
  `std` shall solve any issues.
* OpenCL and MPI modules are moved to
 [vSMCExtra](https://github.com/zhouyan/vSMCExtra).
* `Progress` is no longer a class template.
* `vsmc::Array` is replaced by `std::array`
* The `RngSet` class template is replaced by `RngSetScalar` and `RngSetVector`,
  and the structures `Scalar` and `Vector` are removed.
* The OpenCL module has gone through a complete rework. The library now has
  its own C++ wrapper based on `std::shared_ptr` instead of the official
  (outdated) C++ wrapper.
* The index type `Position` is renamed to `Index`, and it is now a template
  alias to `std::integral_constant`
* The structure `ConstSingleParticle` is removed. All of its occurrence shall
  be replaced by `SingleParticle`. In addition, in most places, where the API
  previously accept `const Particle<T> &` shall now accept `Particle<T> &`
  instead.
* Member functions of SMP bases classes are renamed to use a more consistent
  pattern
  - `pre_processor` -> `eval_pre`
  - `post_processor` -> `eval_post`
  - `initialize_param` -> `eval_param`
  - `initialize_state` -> `eval_sp`
  - `move_state` -> `eval_sp`
  - `monitor_state` -> `eval_sp`
  - `path_state` -> `eval_sp`
  - `path_grid` -> `eval_grid`
* Member function in base value classes `StateMPI` and `StateCL` are renamed
  - `copy_pre_processor` -> `copy_pre`
  - `copy_post_processor` -> `copy_pre`
* Weight classes are renamed
  - `WeightSet` -> `Weight`
  - `WeightSetMPI` -> `WeightMPI`
  The header file `weight_set.hpp` is also renamed to `weight`
* `Weight` (formerly `WeightSet`)'s interface has been overhauled
  - `set_weight` -> `set`
  - `set_log_weight` -> `set_log`
  - `mul_weight` -> `mul`
  - `add_log_weight` -> `add_log`
  - `weight_data` -> `data`
* `Particle::weight_set` renamed to `Particle::weight`
* `weight_set_type` renamed to `weight_type`
* All occurrence of `Rng` are renamed to `RNG`
* `AlignedMemory` and `AlignedAllocator` no longer support
  [jemalloc][jemalloc].

## Removed features

* Modules removed
  - Thread
  - GCD
  - Integrate
  - Core/Adapter
  - Core/StateTuple
  - SMP/Adapter
  - RNG/GSL
  - SMP/CILK
  - SMP/GCD
  - SMP/PPL
  - SMP/STD
  - OpenCL/Adapter
  - Utility/Array
  - Utility/Counter
  - Utility/CString

## New features

* `Monitor::record_data` gets an overload version that return the row pointer.
* `std::unique_ptr` alike wrappers for [Intel MKL][MKL] `VSLStreamStatePtr`,
  `VSLSSTaskPtr`, `VSLConvTaskPtr`, `VSLCorrTaskPtr`, `DFTaskPtr`.

## Bug fixes

* The default resampling threshold when a user defined resampling algorithm is
  provided in `Sampler`'s constructor is fixed to be always resampling, the
  same as for the built-in schemes.

# Changes since v1.1.1

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

# Changes since v1.1.0

## Changed behaviors

* `Sampler` summary member functions, such as `summary_data` etc., now only
  deal with floating point data, such as importance sampling estimates. Integer
  data, such as acceptance counts are dealt with new member functions
  `summary_data_int` etc.
* `Sampler` summary now output raw results of ESS and acceptance counts. They
  are no longer scaled by the number of particles.

## Bug fixes

* `Sampler` now correctly clear size history during initialization

# Changes since v1.0.0

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
[MKL]: https://software.intel.com/en-us/intel-mkl/
[TBB]: https://www.threadingbuildingblocks.org
[jemalloc]: http://www.canonware.com/jemalloc/
