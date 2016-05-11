# Changes in v2.3.0

## Important changes
* `StateMatrix` member `state` is replaced by `operator()`. `Particle` member
  `value` is renamed `state`.
* `Sampler` constructor has been rewritten. It no longer accept arguments about
  resampling method and threshold. Instead, it is now a variadic template and
  pass all of its arguments to `Particle`. The later will take the first
  argument, if it there is one, as the number of particles, and pass the rest
  to value type `T`. Use the new `resample_method` method of `Sampler` to
  configure the behavior of resampling.
* `AlignedAllocator` is renamed to `Allocator`. The old type alias `Allocator`
  is removed. The default alignment is defined by `AlignmentTrait<T>`, which is
  `VSMC_ALIGNMENT` for scalar types and `max(alignof(T), VSMC_ALIGNMENT_MIN)`
  for others.
* New configuration macro `VSMC_CONSTRUCT_SCALAR`. If it is zero (the default),
  then `Allocator` will not zero initialize scalar values when `construct` is
  called without additional arguments.
* The HDF5 module has been rewritten. Now there are only two main functions,
  `hdf5load` and `hdf5store`. The data types are detected based on iterator
  types.

## New features

* New C++ OpenCL RAII classes
* `Sampler` `accept_history` etc., are renamed to `status_history` etc.
* `Sampler` gains new methods, `status_size`, `read_status_history`, and
  `read_status_history_matrix`.
* `StateMatrix` gain `resize(N, dim)` method. Resizing a `StateMatrix` object
  preserve original values in the following sense. Let N and M be the original
  and new sizes, let K and L be the original and new dimensions. Then
  `resize(M, L)` preserve the min(N, M) by min(K, L) matrix at the upper left
  corner of the original. The overloading `resize(N)` assumes the dimension
  does not change. The method `resize_dim` also gains this new behavior of
  preserving values.
* `StateMatrix` can now handle the situation where the length of the input
  `index` to `copy` is not the same as its own size. The results is a resized
  matrix.
* `Particle` can now be resized by various methods
* One can now use resampling algorithms as a `Sampler::move_type` object
  through `ResampleMove`
* New distribution `ArcsineDistribution`
* New generic `MoveSMP` etc., base classes. `MoveTBB<T, Derived` etc., are now
  alias to `MoveSMP<T, Derived, BackendTBB>` etc.
* New vectorized rounding functions
* New RNG type alias, `RNG_64`, `RNGMini` and `RNGMini_64`. And new naming
  scheme, which is more consistent with the others, for AES-NI based RNGs.

## Changed behaviors

* Most methods which take a `MatrixLayout` type template parameter now has that
  parameter as a runtime parameter.
* Most methods that takes output iterators, (including pointers that used in
  this fashion), now returns an iterator in the same way as `std::copy` etc.
* `Vector` is now defined to be `std::vector<T, Allocator<T>>`
* `Sampler::resample_scheme` is now renamed `resample_method`
* `Particle::resample` is removed

## Removed features

* `Monitor::read_record_matrix` overload which takes iterators to iterators is
  removed
* `StateMatrix::read_state_matrix` overload which takes iterators to iterators
  is removed
* MKL RAII classes no long has the `reset` method with same arguments as the
  constructors. Use the constructor and move semantics instead.
* `Monitor::index_data` and `Monitor::record_data` are removed
* `AlignedVector` is removed
* `hdf5store` is now overloaded for `Monitor`
* `Sampler::mcmc_type` is removed. It was exactly the same as
  `Sampler::move_type`. Unless the user code explicitly use the name
  `mcmc_type`, nothing shall break.

# Changes in v2.2.0

## New features

* `SingleParticle` now as iterator-like behaviors.
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
  functions to be static and the class stateless. Unless you write your
  allocator using this template, there is no need to change client code.
* `MatrixOrder` is now an alias to `MatrixLayout`.
* CBLAS and LAPACK are now required dependency.

## Removed features

* `U01LRDistribution` is removed, while `U01CCDistribution` etc., remain.
* `UniformRealLRDistribution` and `UniformRealCCDistribution` etc., are
  removed.
* Path sampling support is removed. It can be easily done through the more
  general `Monitor`. See the GMM example.

## Documentation
http://zhouyan.github.io/vSMCDoc/v2.2.0/

# Changes in v2.1.0

## Important changes

The mutation (MCMC) steps are now performed after the resampling of
initialization step.

## Removed features

* OpenCL and MPI modules are removed for now. It is planned that they will be
  added back in future with a new interface.

## Documentation
http://zhouyan.github.io/vSMCDoc/v2.1.0/

# Changes in v2.0.0

The version 2.0.0 is a major restructuring of the library since the initial
release. Its documentation is still been updated. For the time being, one can
keep using the old JSS release or the reference manual below.

The most important change is that C++11, both language and libraries are now
required.

## Documentation
http://zhouyan.github.io/vSMCDoc/v2.0.0/

[HDF5]: http://www.hdfgroup.org/HDF5/
[MKL]: https://software.intel.com/en-us/intel-mkl/
[TBB]: https://www.threadingbuildingblocks.org
