# Examples of the vSMC library

This directory contains examples of the [vSMC][vSMC] library. See the
`README.md` files in each subdirectory for a description of the examples and
references.

In many examples, multiple executables that implement the same algorithm may be
built. They differ in the parallel programming models used. These executables
follow a naming scheme as `<example>_<alg>_<prl>` where `<example>` is the name
of the example, same as the subdirectory name, e.g., `gmm` for the Gaussian
mixture model example; `<alg>` is the name of the algorithm, e.g., `smc` for
the sequential Monte Carlo algorithm and `<prl>` is the programming model, for
example `tbb` for the [Intel TBB][Intel TBB] parallel programming library.
Thus, `gmm_smc_tbb` is an executable that employs the [Intel TBB][Intel TBB]
parallelization and implements the sequential Monte Carlo algorithm for the
Gaussian mixture model example.

For some examples, `<alg>` and/or `<prl>` may be missing. For example, the
particle filter example, found in the `pf` subdirectory only has one algorithm
and thus the names are formed as `pf_<prl>` where `<prl>` are `cl`, `tbb`, etc.
The RNG examples, found in the `rng` subdirectory are demos and tests of the
RNG module of the [vSMC][vSMC] library and there are no `<alg>` or `<prl>`
suffices.

# License

All example sources are distributed with a 2-clause BSD license which can
be found in the `LICENSE` file distributed with the source.

[Intel TBB]: http://threadingbuildingblocks.org/
[CMake]: http://www.cmake.org/
[vSMC]: https://github.com/zhouyan/vSMC
