#ifndef VSMC_HPP
#define VSMC_HPP

#include <vsmc/internal/config.hpp>

#include <vsmc/core/sampler.hpp>
#include <vsmc/core/particle.hpp>
#include <vsmc/core/monitor.hpp>
#include <vsmc/core/path.hpp>

#include <vsmc/smp/sequential.hpp>
#include <vsmc/smp/adapter.hpp>

#if VSMC_USE_CILK
#include <vsmc/smp/parallel_cilk.hpp>
#endif

#if VSMC_USE_GCD
#include <vsmc/smp/parallel_gcd.hpp>
#endif

#if VSMC_USE_OMP
#include <vsmc/smp/parallel_omp.hpp>
#endif

#if VSMC_USE_PPL
#include <vsmc/smp/parallel_ppl.hpp>
#endif

#if VSMC_USE_TBB
#include <vsmc/smp/parallel_tbb.hpp>
#endif

#if VSMC_USE_STD
#include <vsmc/smp/parallel_std.hpp>
#endif

#if VSMC_USE_CL
#include <vsmc/opencl/parallel_cl.hpp>
#endif

#endif // VSMC_HPP

/// \defgroup Core Core
/// \brief Constructing samplers with operations on the whole particle set

/// \defgroup Resampling Resampling
/// \ingroup Core
/// \brief Various resampling methods

/// \defgroup SMP Symmetric Multiprocessing
/// \brief Single threaded and parallel samplers using SMP implementations

/// \defgroup Base Base Dispatcher
/// \ingroup SMP
/// \brief Base class templates that dispatch computing tasks

/// \defgroup Adapter Adapter
/// \ingroup SMP
/// \brief Adapter class templates for constructing concrete objects

/// \defgroup Implementation Implementation
/// \ingroup SMP
/// \brief Implementation class templates that parallelize user callbacks

/// \defgroup Sequential Sequential
/// \ingroup Implementation
/// \brief Single threaded sampler
///
/// \defgroup GCD Apple Grand Central Dispatch
/// \ingroup Implementation
/// \brief Parallelized sampler with Apple Grand Central Dispatch

/// \defgroup CILK Intel Cilk Plus
/// \ingroup Implementation
/// \brief Parallelized sampler with Intel Cilk Plus

/// \defgroup OMP OpenMP
/// \ingroup Implementation
/// \brief Parallelized sampler with OpenMP

/// \defgroup PPL Microsoft Parallel Pattern Library
/// \ingroup Implementation
/// \brief Parallelized sampler with Microsoft Parallel Pattern Library

/// \defgroup STD C++11 Concurrency
/// \ingroup Implementation
/// \brief Parallelized sampler with C++11 concurrency

/// \defgroup TBB Intel Threading Buidling Block
/// \ingroup Implementation
/// \brief Parallelized sampler with Intel Threading Building Block

/// \defgroup CL OpenCL
/// \brief Parallelized sampler with OpenCL

/// \defgroup Utility Utility
/// \brief Utilities independent of other part of the library

/// \defgroup Dispatch Apple Grand Central Dispatch
/// \ingroup Utility
/// \brief Apple Grand Central Dispatch dispatch queue utilities

/// \defgroup Integrate Integration
/// \ingroup Utility
/// \brief Importance sampling and numerical integration

/// \defgroup OpenCL OpenCL
/// \ingroup Utility
/// \brief OpenCL device management utilities

/// \defgroup Random Random
/// \ingroup Utility
/// \brief Generating distribution random variates using Random123

/// \defgroup STDTBB C++11 Concurrency
/// \ingroup Utility
/// \brief C++11 concurrency for implementing simple parallel algorithms
