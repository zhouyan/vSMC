#ifndef VSMC_HPP
#define VSMC_HPP

#include <vsmc/internal/config.hpp>

#include <vsmc/core/sampler.hpp>
#include <vsmc/core/particle.hpp>
#include <vsmc/core/monitor.hpp>
#include <vsmc/core/path.hpp>

#include <vsmc/helper/sequential.hpp>
#include <vsmc/helper/adapter.hpp>

#if VSMC_USE_CILK
#include <vsmc/helper/parallel_cilk.hpp>
#endif // VSMC_USE_CILK

#if VSMC_USE_CL
#include <vsmc/helper/parallel_cl.hpp>
#endif // VSMC_USE_CL

#if VSMC_USE_OMP
#include <vsmc/helper/parallel_omp.hpp>
#endif // VSMC_USE_OMP

#if VSMC_USE_TBB
#include <vsmc/helper/parallel_tbb.hpp>
#endif // VSMC_USE_TBB

#if VSMC_USE_STD
#include <vsmc/helper/parallel_std.hpp>
#endif // VSMC_USE_STD

#endif // VSMC_HPP

/// \defgroup Core Core
/// \brief Constructing samplers with operations on the whole particle set

/// \defgroup Resampling Resampling
/// \ingroup Core
/// \brief Various resampling methods

/// \defgroup Helper Helper
/// \brief Single threaded and parallel samplers based on a single particle
/// operations

/// \defgroup Base Dispatcher
/// \ingroup Helper
/// \brief Base class templates that dispatch computing tasks based on
/// implementations

/// \defgroup Adapter Adapter
/// \ingroup Helper
/// \brief Adapter class templates for constructing concrete objects from
/// implementation base class templates

/// \defgroup Implementation Implementation
/// \ingroup Helper
/// \brief Implementation class templates that parallelize user defined
/// computing tasks

/// \defgroup Sequential Sequential
/// \ingroup Implementation
/// \brief Single threaded sampler

/// \defgroup STD C++11 Multithread Support
/// \ingroup Implementation
/// \brief Parallelized samplers with C++11 multithread support

/// \defgroup TBB Intel Threading Buidling Block
/// \ingroup Implementation
/// \brief Parallelized samplers with Intel TBB

/// \defgroup CILK Intel Cilk Plus
/// \ingroup Implementation
/// \brief Parallelized samplers with Intel Cilk Plus

/// \defgroup OpenMP OpenMP
/// \ingroup Implementation
/// \brief Parallelized samplers with OpenMP

/// \defgroup OpenCL OpenCL
/// \ingroup Implementation
/// \brief Parallelized sampler with OpenCL

/// \defgroup Utility Utility
/// \brief Utilities

/// \defgroup RNG RNG
/// \ingroup Utility
/// \brief Random Number Generation utitilites

/// \defgroup Thread Thread
/// \ingroup Utility
/// \brief C++11 Thread utilities
