#ifndef VSMC_HPP
#define VSMC_HPP

#include <vsmc/internal/config.hpp>

#include <vsmc/core/sampler.hpp>
#include <vsmc/core/particle.hpp>
#include <vsmc/core/monitor.hpp>
#include <vsmc/core/path.hpp>

#include <vsmc/helper/sequential.hpp>

#if VSMC_USE_OMP
#include <vsmc/helper/parallel_omp.hpp>
#endif // VSMC_USE_OMP

#if VSMC_USE_TBB
#include <vsmc/helper/parallel_tbb.hpp>
#endif // VSMC_USE_TBB

#if VSMC_USE_CILK
#include <vsmc/helper/parallel_cilk.hpp>
#endif // VSMC_USE_CILK

#if VSMC_USE_CL
#include <vsmc/helper/parallel_cl.hpp>
#endif // VSMC_USE_CL

#if VSMC_HAS_CXX11LIB_CHRONO
#include <vsmc/timer/chrono_timer.hpp>
#endif // VSMC_HAS_CXX11LIB_CHRONO

#endif // VSMC_HPP

/// \defgroup Core Core
/// \brief Constructing samplers with operations on the whole particle set

/// \defgroup Helper Helper
/// \brief Constructing samplers with operations on a single particle

/// \defgroup Timer Timer
/// \brief Timer used by Helper classes
