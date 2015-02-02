# Changes since v1.0.0

## New features

* Support for [jemalloc][jemalloc] in `utility/aligned_memory.hpp`
* `AlignedMemory` and `AlignedAllocator` default to [jemalloc][jemalloc] if it
  is available
* Store `Particle<T>` object in [HDF5][HDF5] format

[HDF5]: http://www.hdfgroup.org/HDF5/
[jemalloc]: http://www.canonware.com/jemalloc/
