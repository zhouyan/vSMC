# Changes since v1.0.0

* Add support for [jemalloc][jemalloc] in `utility/aligned_memory.hpp`
* `AlignedMemory` and `AlignedAllocator` default to [jemalloc][jemalloc] if it
  is available.

[jemalloc]: http://www.canonware.com/jemalloc/
