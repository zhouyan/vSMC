# Particle filter

This example implements the particle filter algorithm for almost constant
velocity model found in:

Johansen, A. M. (2009). SMCTC: sequential Monte Carlo in C++. Journal of
Statistical Software, 30(6), 1-41.

## Extras

This example also serves as a compiler stress test. It build a few variants of
the implementations

- `pf_cl`: OpenCL implementation
- `pf_matrix` and `pf_tuple`: Using `vsmc::StateMatrix` and `vsmc::StateTuple`
  as base state classes, respectively
- `pf_<matrix|tuple>_mpi`: Using MPI parallelization

Therefore there are five base implementations, `pf_cl`, `pf_matrix`,
`pf_tuple`, `pf_matrix_mpi`, `pf_tuple_mpi`. The later four also come with
different SMP parallelizations such as `tbb`.
