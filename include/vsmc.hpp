#ifndef VSMC_HPP
#define VSMC_HPP

#include <vsmc/core/sampler.hpp>
#include <vsmc/core/particle.hpp>
#include <vsmc/core/monitor.hpp>
#include <vsmc/core/path.hpp>

#include <vsmc/helper/timer/null.hpp>
#include <vsmc/helper/sequential.hpp>

#ifdef VSMC_USE_OMP
#include <vsmc/helper/parallel_omp.hpp>
#endif // VSMC_USE_OMP

#ifdef VSMC_USE_TBB
#include <vsmc/helper/parallel_tbb.hpp>
#endif // VSMC_USE_TBB

#ifdef VSMC_USE_CL
#include <vsmc/helper/parallel_cl.hpp>
#endif // VSMC_USE_CL

#endif // VSMC_HPP

/// \defgroup Core Core
/// \brief Constructing samplers with operations on the whole particle set

/// \defgroup Helper Helper
/// \brief Constructing samplers with operations on a single particle

/// \defgroup Timer Timer
/// \ingroup Helper
/// \brief Timer used by Helper classes
