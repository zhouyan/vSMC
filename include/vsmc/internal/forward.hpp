#ifndef VSMC_INTERNAL_FORWARD_HPP
#define VSMC_INTERNAL_FORWARD_HPP

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/defines.hpp>

#include <cstddef>

#define VSMC_DEFINE_SMP_FORWARD(Name) \
template <typename T, typename Derived = Virtual> class Initialize##Name;    \
template <typename T, typename Derived = Virtual> class Move##Name;          \
template <typename T, typename Derived = Virtual> class MonitorEval##Name;   \
template <typename T, typename Derived = Virtual> class PathEval##Name;

namespace vsmc {

// Template default arguments
struct Virtual;
struct NullType;

// std::tuple function recursion
template <std::size_t> struct Position {};

// Core classes
template <typename T> class Sampler;
template <typename T> class Particle;
template <typename T> class Monitor;
template <typename T> class Path;
template <typename T> class SingleParticle;
template <typename T> class ConstSingleParticle;
template <typename T> class SingleParticleBase;
template <typename T> class ConstSingleParticleBase;
class WeightSet;
class NormalizingConstant;

// SMP states
template <MatrixOrder Order, std::size_t Dim, typename T> class StateMatrix;
#if VSMC_HAS_CXX11LIB_TUPLE
template <MatrixOrder Order, typename T, typename... Types> class StateTuple;
#endif

// OpenCL state
struct CLDefault;
template <std::size_t StateSize, typename FPType, typename ID = CLDefault>
class StateCL;

} // namesapce vsmc

#endif // VSMC_INTERNAL_FORWARD_HPP
