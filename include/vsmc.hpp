#ifndef VSMC_HPP
#define VSMC_HPP

#include <vsmc/core/sampler.hpp>
#include <vsmc/core/particle.hpp>
#include <vsmc/core/monitor.hpp>
#include <vsmc/core/path.hpp>

#include <vsmc/helper/single_particle.hpp>
#include <vsmc/helper/state_base.hpp>

#include <vsmc/helper/sequential.hpp>

#ifdef VSMC_USE_TBB
#include <vsmc/helper/parallel_tbb.hpp>
#endif // VSMC_USE_TBB

#ifdef VSMC_USE_CL
#include <vsmc/helper/paralle_cl.hpp>
#endif // VSMC_USE_CL

#endif // VSMC_HPP

/// \defgroup Core Core
///
/// \defgroup Helper Helper
///
/// \defgroup Sequential Sequential
/// \ingroup Helper
///
/// \defgroup TBB Intel Threading Buidling Block
/// \ingroup Helper
///
/// \defgroup OpenCL OpenCL
/// \ingroup Helper
///
/// \defgroup RNG Random number generating
