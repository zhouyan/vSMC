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

[HDF5]: http://www.hdfgroup.org/HDF5/
[jemalloc]: http://www.canonware.com/jemalloc/
