/// \defgroup Config Configuration
/// \brief Configuration macros and default values if undefined

/// \defgroup Compiler Compiler configration
/// \brief Compiler related configuration macros. Default value varies with
/// different compilers
/// \ingroup Config

/// \defgroup Definitions Constants definitions
/// \brief Enumerators and macro definitions

/// \defgroup Traits Traits
/// \brief Trait classes

/// \defgroup Core Core
/// \brief Constructing samplers with operations on the whole particle set

/// \defgroup Resample Resampling algorithms
/// \brief Resampling algorithm functor classes

/// \defgroup Adapter Adapter
/// \brief Adapter class templates for constructing concrete objects

/// \defgroup Integrate Integration
/// \brief Numerical integration

/// \defgroup MPI Message Passing Interface
/// \brief Parallel samplers using MPI

/// \defgroup OpenCL OpenCL
/// \brief Parallel sampler using OpenCL

/// \defgroup SMP Symmetric Multiprocessing
/// \brief Parallel samplers using multi-threading on SMP architecture

/// \defgroup CILK Intel Cilk Plus
/// \ingroup SMP
/// \brief Parallel samplers using Intel Cilk Plus

/// \defgroup GCD Grand Central Dispatch
/// \ingroup SMP
/// \brief Parallel samplers using Apple GCD

/// \defgroup OMP OpenMP
/// \ingroup SMP
/// \brief Parallel samplers using OpenMP

/// \defgroup PPL Parallel Pattern Library
/// \ingroup SMP
/// \brief Parallel samplers using Microsoft PPL

/// \defgroup SEQ Sequential
/// \ingroup SMP
/// \brief Sequential samplers

/// \defgroup STD C++11 concurrency
/// \ingroup SMP
/// \brief Parallel samplers using C++11 concurrency

/// \defgroup TBB Intel Threading Building Blocks
/// \ingroup SMP
/// \brief Parallel samplers using Intel TBB

/// \defgroup Math Mathematics
/// \brief Mathematical utilities

/// \defgroup Constants Constants
/// \ingroup Math
/// \brief Mathematical constants

/// \defgroup RNG Random number generating
/// \brief Utilities for managing RNG engines

/// \defgroup R123RNG Random123 RNG
/// \ingroup RNG
/// \brief Random number generating using Random123 RNG

/// \defgroup MKLRNG MKL RNG
/// \ingroup RNG
/// \brief Random number generating using MKL RNG

/// \defgroup Utility Utility
/// \brief Utilities independent of other part of the library

/// \defgroup Backup Object backup
/// \ingroup Utility
/// \brief Backup and restore objects

/// \defgroup Dispatch Grand Central Dispatch
/// \ingroup Utility
/// \brief C++ wrapper of Apple GCD

/// \defgroup HDF5Save HDF5 objects saving
/// \ingroup Utility
/// \brief Functions for saving objects in HDF5 format

/// \defgroup Option Program option
/// \ingroup Utility
/// \brief Process program command line options

/// \defgroup STDTBB STDTBB
/// \ingroup Utility
/// \brief Parallel constructs using C++11 concurrency

/// \defgroup StopWatch Stop watch
/// \ingroup Utility
/// \brief Utilities for measuring the time of procedures

/// \defgroup TBBOp TBB/STDTBB operators
/// \ingroup Utility
/// \brief Operators for Intel TBB and STDTBB

/// \defgroup Tuple Tuple manipulation
/// \ingroup Utility
/// \brief Classes for manipulating C++11 tuple

#ifndef VSMC_HPP
#define VSMC_HPP

#include <vsmc/core/adapter.hpp>
#include <vsmc/core/monitor.hpp>
#include <vsmc/core/normalizing_constant.hpp>
#include <vsmc/core/particle.hpp>
#include <vsmc/core/path.hpp>
#include <vsmc/core/sampler.hpp>
#include <vsmc/core/single_particle.hpp>
#include <vsmc/core/weight_set.hpp>

#include <vsmc/cxx11/cmath.hpp>
#include <vsmc/cxx11/functional.hpp>
#include <vsmc/cxx11/random.hpp>
#include <vsmc/cxx11/type_traits.hpp>

#include <vsmc/integrate/is_integrate.hpp>
#include <vsmc/integrate/nintegrate_base.hpp>
#include <vsmc/integrate/nintegrate_newton_cotes.hpp>

#include <vsmc/math/constants.hpp>

// #include <vsmc/mpi/backend_mpi.hpp>
// #include <vsmc/mpi/mpi_manager.hpp>

// #include <vsmc/opencl/adapter.hpp>
// #include <vsmc/opencl/backend_cl.hpp>
// #include <vsmc/opencl/cl_manager.hpp>
// #include <vsmc/opencl/cl_manip.hpp>
// #include <vsmc/opencl/cl_query.hpp>

#include <vsmc/resample/basic.hpp>
#include <vsmc/resample/multinomial.hpp>
#include <vsmc/resample/residual.hpp>
#include <vsmc/resample/residual_stratified.hpp>
#include <vsmc/resample/residual_systematic.hpp>
#include <vsmc/resample/stratified.hpp>
#include <vsmc/resample/systematic.hpp>

#include <vsmc/rng/gammak1.h>
// #include <vsmc/rng/mkl_rng.hpp>
#include <vsmc/rng/normal01.h>
#include <vsmc/rng/rng_set.hpp>
#include <vsmc/rng/seed.hpp>
#include <vsmc/rng/u01.h>
#include <vsmc/rng/uniform_real_distribution.hpp>
#include <vsmc/rng/urng.h>

#include <vsmc/smp/adapter.hpp>
#include <vsmc/smp/backend_base.hpp>
// #include <vsmc/smp/backend_cilk.hpp>
// #include <vsmc/smp/backend_gcd.hpp>
// #include <vsmc/smp/backend_omp.hpp>
// #include <vsmc/smp/backend_ppl.hpp>
#include <vsmc/smp/backend_seq.hpp>
// #include <vsmc/smp/backend_std.hpp>
// #include <vsmc/smp/backend_tbb.hpp>
#include <vsmc/smp/iterator.hpp>
#include <vsmc/smp/state_matrix.hpp>
// #include <vsmc/smp/state_tuple.hpp>

#include <vsmc/utility/backup.hpp>
// #include <vsmc/utility/dispatch.hpp>
// #include <vsmc/utility/hdf5_save.hpp>
#include <vsmc/utility/program_option.hpp>
// #include <vsmc/utility/stdtbb.hpp>
#include <vsmc/utility/stop_watch.hpp>
#include <vsmc/utility/tbb_op.hpp>
// #include <vsmc/utility/tuple_manip.hpp>

#endif // VSMC_HPP
