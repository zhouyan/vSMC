#ifndef VSMC_INTERNAL_FORWARD_HPP
#define VSMC_INTERNAL_FORWARD_HPP

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/defines.hpp>

#include <cstddef>

/// \cond HIDDEN_SYMBOLS

#define VSMC_DEFINE_SMP_FORWARD(Name) \
template <typename> class State##Name;                                       \
template <typename> class WeightSet##Name;                                   \
class NormalizingConstant##Name;                                             \
template <typename, typename D = Virtual> class Initialize##Name;            \
template <typename, typename D = Virtual> class Move##Name;                  \
template <typename, typename D = Virtual> class MonitorEval##Name;           \
template <typename, typename D = Virtual> class PathEval##Name;              \
template <typename> class NIntegrate##Name;

namespace vsmc {

// Template default arguments
struct Virtual;
struct NullType;

// std::tuple function recursion
template <std::size_t> struct Position {};

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

// SMP states
template <MatrixOrder, std::size_t, typename> class StateMatrix;
#if VSMC_HAS_CXX11LIB_TUPLE
template <MatrixOrder, typename, typename...> class StateTuple;
#endif

// OpenCL state
struct CLDefault;
template <std::size_t, typename, typename ID = CLDefault> class StateCL;

} // namesapce vsmc

/// \endcond HIDDEN_SYMBOLS

#endif // VSMC_INTERNAL_FORWARD_HPP
