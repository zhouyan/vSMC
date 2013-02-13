#ifndef VSMC_INTERNAL_FORWARD_HPP
#define VSMC_INTERNAL_FORWARD_HPP

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/defines.hpp>

#include <cstddef>

/// \cond HIDDEN_SYMBOLS

#define VSMC_DEFINE_SMP_FORWARD(Name)                                         \
namespace vsmc {                                                              \
    template <typename> class State##Name;                                    \
    template <typename, typename D = VBase> class Initialize##Name;           \
    template <typename, typename D = VBase> class Move##Name;                 \
    template <typename, typename D = VBase> class MonitorEval##Name;          \
    template <typename, typename D = VBase> class PathEval##Name;             \
    template <typename> class Numeric##Name;                                  \
}

namespace vsmc {

// Placeholders
struct CBase;
struct VBase;
struct NullType;
struct ScalarRng;
struct VectorRng;

// Utilities
class Seed;
template <typename, typename> class RngSet;

class ImportanceSampling1;
class ImportanceSamplingD;
template <typename> class NumericBase;
template <unsigned, template <typename> class> class NumericNewtonCotes;

class DispatchQueue;

template <typename> class BlockedRange;
class ThreadGuard;
class ThreadInfo;

struct CLDefault;
class CLConfigure;
class CLQuery;
template <typename> class CLManager;

// Adapter
template <typename, template <typename, typename> class, typename B = CBase>
         class InitializeAdapter;
template <typename, template <typename, typename> class, typename B = CBase>
         class MoveAdapter;
template <typename, template <typename, typename> class, typename B = CBase>
         class MonitorEvalAdapter;
template <typename, template <typename, typename> class, typename B = CBase>
         class PathEvalAdapter;

// Core
template <typename ResType> class Resample;
template <typename> class WeightSet;
template <typename> class Sampler;
template <typename> class Particle;
template <typename> class Monitor;
template <typename> class Path;
template <typename, template <typename> class> class PathGeometry;
template <typename, template <typename> class> class ParticleIterator;
template <typename> class SingleParticle;
template <typename> class SingleParticleBase;
template <typename> class ConstSingleParticle;
template <typename> class ConstSingleParticleBase;

// OpenCL
template <std::size_t, typename, typename ID = VSMC_CL_DEFAULT_ID>
class StateCL;
template <typename, typename B = CBase> class InitializeCL;
template <typename, typename B = CBase> class MoveCL;
template <typename, typename B = CBase> class MonitorEvalCL;
template <typename, typename B = CBase> class PathEvalCL;

// SMP State
template <MatrixOrder, std::size_t, typename> class StateMatrixBase;
template <MatrixOrder, std::size_t, typename> class StateMatrix;
#if VSMC_HAS_CXX11_VARIADIC_TEMPLATES 
template <MatrixOrder, typename, typename...> class StateTupleBase;
template <MatrixOrder, typename, typename...> class StateTuple;
#endif

// SMP Base
template <typename, typename> class InitializeBase;
template <typename, typename> class MoveBase;
template <typename, typename> class MonitorEvalBase;
template <typename, typename> class PathEvalBase;

} // namesapce vsmc

// SMP Implementations
VSMC_DEFINE_SMP_FORWARD(SEQ)
VSMC_DEFINE_SMP_FORWARD(CILK)
VSMC_DEFINE_SMP_FORWARD(GCD)
VSMC_DEFINE_SMP_FORWARD(OMP)
VSMC_DEFINE_SMP_FORWARD(PPL)
VSMC_DEFINE_SMP_FORWARD(STD)
VSMC_DEFINE_SMP_FORWARD(TBB)

/// \endcond HIDDEN_SYMBOLS

#endif // VSMC_INTERNAL_FORWARD_HPP
