#ifndef VSMC_HPP
#define VSMC_HPP

#include <vsmc/internal/config.hpp>

#include <vsmc/core/sampler.hpp>
#include <vsmc/core/particle.hpp>
#include <vsmc/core/monitor.hpp>
#include <vsmc/core/path.hpp>

#include <vsmc/helper/sequential.hpp>
#include <vsmc/helper/adaptor.hpp>

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
/// \brief Constructing samplers with operations on a single particle

/// \defgroup Timer Timer
/// \ingroup Helper
/// \brief Timer used by Helper classes

/// \defgroup Sequential Sequential
/// \ingroup Helper
/// \brief Single threaded sampler

/// \defgroup STDThread C++11 Multithread Support
/// \ingroup Helper
/// \brief Parallelized samplers with C++11 multithread support

/// \defgroup TBB Intel Threading Buidling Block
/// \ingroup Helper
/// \brief Parallelized samplers with Intel TBB

/// \defgroup CILK Intel Cilk Plus
/// \ingroup Helper
/// \brief Parallelized samplers with Intel Cilk Plus

/// \defgroup OpenMP OpenMP
/// \ingroup Helper
/// \brief Parallelized samplers with OpenMP

/// \defgroup OpenCL OpenCL
/// \ingroup Helper
/// \brief Parallelized sampler with OpenCL
