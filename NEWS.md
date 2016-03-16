# Changes in v2.2.0

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

## Documentation
http://zhouyan.github.io/vSMCDoc/v2.2.0/

# Changes in v2.1.0

## Important changes

The mutation (MCMC) steps are now performed after the resampling of
initialization step.

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
