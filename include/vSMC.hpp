#ifndef V_SMC_HPP
#define V_SMC_HPP

#include <vSMC/core/sampler.hpp>
#include <vSMC/core/particle.hpp>
#include <vSMC/core/monitor.hpp>
#include <vSMC/core/path.hpp>

#include <vSMC/helper/single_particle.hpp>
#include <vSMC/helper/state_base.hpp>

#include <vSMC/helper/sequential.hpp>

#ifdef V_SMC_USE_TBB
#include <vSMC/helper/parallel_tbb.hpp>
#endif // V_SMC_USE_TBB

#ifdef V_SMC_USE_CL
#include <vSMC/helper/paralle_cl.hpp>
#endif // V_SMC_USE_CL

#endif // V_SMC_HPP

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
