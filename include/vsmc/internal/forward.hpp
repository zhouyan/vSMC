//============================================================================
// include/vsmc/internal/forward.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distributed under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_INTERNAL_FORWARD_HPP
#define VSMC_INTERNAL_FORWARD_HPP

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/defines.hpp>

#define VSMC_DEFINE_SMP_FORWARD(Name) \
template <typename T, typename = Virtual> class Initialize##Name;            \
template <typename T, typename = Virtual> class Move##Name;                  \
template <typename T, typename = Virtual> class MonitorEval##Name;           \
template <typename T, typename = Virtual> class PathEval##Name;

namespace vsmc {

// Template default arguments
struct Virtual;
struct NullType;

// Core classes
template <typename> class Sampler;
template <typename> class Particle;
template <typename> class Monitor;
template <typename> class Path;
template <typename> class SingleParticle;
template <typename> class ConstSingleParticle;
template <typename> class SingleParticleBase;
template <typename> class ConstSingleParticleBase;
class WeightSet;
class NormalizingConstant;

// SMP states
template <MatrixOrder, std::size_t, typename> class StateMatrix;
#if VSMC_HAS_CXX11LIB_TUPLE
template <MatrixOrder, typename, typename...> class StateTuple;
#endif

// OpenCL state
struct CLDefault;
template <std::size_t, typename, typename = CLDefault> class StateCL;

} // namesapce vsmc

#endif // VSMC_INTERNAL_FORWARD_HPP
