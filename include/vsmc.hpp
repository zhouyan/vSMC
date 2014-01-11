#ifndef VSMC_HPP
#define VSMC_HPP

#include <vsmc/core/sampler.hpp>
#include <vsmc/smp/adapter.hpp>
#include <vsmc/smp/backend_seq.hpp>
// #include <vsmc/smp/backend_cilk.hpp>
// #include <vsmc/smp/backend_gcd.hpp>
// #include <vsmc/smp/backend_omp.hpp>
// #include <vsmc/smp/backend_ppl.hpp>
// #include <vsmc/smp/backend_std.hpp>
// #include <vsmc/smp/backend_tbb.hpp>
// #include <vsmc/mpi/backend_mpi.hpp>
// #include <vsmc/opencl/backend_cl.hpp>
#include <vsmc/utility/integrate/numeric_seq.hpp>
// #include <vsmc/utility/integrate/numeric_cilk.hpp>
// #include <vsmc/utility/integrate/numeric_gcd.hpp>
// #include <vsmc/utility/integrate/numeric_omp.hpp>
// #include <vsmc/utility/integrate/numeric_ppl.hpp>
// #include <vsmc/utility/integrate/numeric_std.hpp>
// #include <vsmc/utility/integrate/numeric_tbb.hpp>

#endif // VSMC_HPP

/// \defgroup Config Configuration
/// \brief Configuration macros and default values if undefined

/// \defgroup Compiler Compiler configration
/// \brief Compiler related configuration macros. Default value varies with
/// different compilers
/// \ingroup Config

/// \defgroup Core Core
/// \brief Constructing samplers with operations on the whole particle set

/// \defgroup Resample Resampling algorithms
/// \brief Resampling algorithm functor classes

/// \defgroup Adapter Adapter
/// \brief Adapter class templates for constructing concrete objects

/// \defgroup OpenCL OpenCL
/// \brief Parallel sampler using OpenCL

/// \defgroup OpenCLRNG OpenCL random number generating
/// \ingroup OpenCL
/// \brief Generating random numbers on OpenCL devices

/// \defgroup SMP Symmetric Multiprocessing
/// \brief Parallel samplers using multi-threading on SMP architecture

/// \defgroup Cilk Intel Cilk Plus
/// \ingroup SMP
/// \brief Parallel samplers using Intel Cilk Plus

/// \defgroup GCD Apple Grand Central Dispatch
/// \ingroup SMP
/// \brief Parallel samplers using Apple GCD

/// \defgroup OMP OpenMP
/// \ingroup SMP
/// \brief Parallel samplers using OpenMP

/// \defgroup PPL Microsoft Parallel Pattern Library
/// \ingroup SMP
/// \brief Parallel samplers using Microsoft PPL

/// \defgroup SEQ Sequential
/// \ingroup SMP
/// \brief Sequential samplers

/// \defgroup STD STDTBB
/// \ingroup SMP
/// \brief Parallel samplers using C++11 concurrency

/// \defgroup TBB Intel Threading Building Blocks
/// \ingroup SMP
/// \brief Parallel samplers using Intel TBB

/// \defgroup MPI Message Passing Interface
/// \brief Parallel samplers using MPI

/// \defgroup Traits Traits
/// \brief Trait classes

/// \defgroup Utility Utility
/// \brief Utilities independent of other part of the library

/// \defgroup Backup Object Backup
/// \ingroup Utility
/// \brief Backup and restore objects

/// \defgroup CLUtility OpenCL Utilities
/// \ingroup Utility
/// \brief Utilities for managing OpenCL objects

/// \defgroup Dispatch Apple GCD Utilities
/// \ingroup Utility
/// \brief Utilities for managing Apple GCD queues

/// \defgroup Integrate Integration
/// \ingroup Utility
/// \brief Numerical integration

/// \defgroup Iterator Iterator
/// \ingroup Utility
/// \brief Non-standard iterators

/// \defgroup RNG RNG
/// \ingroup Utility
/// \brief Utilities for managing independent RNG engines

/// \defgroup STDTBB STDTBB
/// \ingroup Utility
/// \brief Parallel constructs using C++11 concurrency

/// \defgroup StopWatch Stop Watch
/// \ingroup Utility
/// \brief Utilities for measuring the time of procedures

/// \defgroup TBBOp TBB/STDTBB operators
/// \ingroup Utility
/// \brief Operators for Intel TBB and STDTBB

/// \defgroup Tuple Tuple Manipulation
/// \ingroup Utility
/// \brief Classes for manipulating C++11 tuple

/// \defgroup MRW Metropolis Random Walk
/// \brief Metropolis Random Walk kernels
