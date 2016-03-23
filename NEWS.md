# Changes in v2.3.0

## New features

* C++ OpenCL RAII classes
* `Sampler` gains new methods, `accept_size`, `read_accept_history`,
  `read_accept_history_list`, and `read_accept_history_matrix`.

## Changed behaviors

* `Monitor::read_record_matrix` overload which takes iterators to iterators is
  renamed to `read_record_list`.
* `StateMatrix::read_state_matrix` overload which takes iterators to iterators
  is renamed to `read_record_list`
* Most methods which take a `MatrixLayout` type template parameter now has that
  parameter as a runtime parameter.
* Most methods that takes output iterators, (including pointers that used in
  this fashion), now returns an iterator in the same way as `std::copy` etc.
* `Allocator` is now always an alias to `AlignedAllocator`. The alignment is 32
  for scalar types and `max(alignof(T), 16)` for others.
* `Vector` is now defined to be `std::vector<T, Allocator<T>`

## Removed features

* MKL RAII classes no long has the `reset` method with same arguments as the
  constructors. Use the constructor and move semantics instead. 
* `Monitor::index_data` and `Monitor::record_data` are removed
* `AlignedVector` is removed

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
