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
[TBB]: https://www.threadingbuildingblocks.org
[jemalloc]: http://www.canonware.com/jemalloc/
