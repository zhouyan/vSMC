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
template <typename> class Numeric##Name;

namespace vsmc {

// Placeholders
struct Virtual;
struct NullType;
struct ScalarRng;
struct VectorRng;

// Utilities
template <typename> class SeedGenerator;
template <typename, typename> class RngSet;

class ISIntegrate;
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
template <typename, typename, typename> class InitializeAdapterBase;
template <typename, typename, typename> class MoveAdapterBase;
template <typename, typename, typename> class MonitorEvalAdapterBase;
template <typename, typename, typename> class PathEvalAdapterBase;

template <typename, template <typename, typename> class, typename F = NullType>
class InitializeAdapter;
template <typename, template <typename, typename> class, typename F = NullType>
class MoveAdapter;
template <typename, template <typename, typename> class, typename F = NullType>
class MonitorEvalAdapter;
template <typename, template <typename, typename> class, typename F = NullType>
class PathEvalAdapter;

// Core
class WeightSet;
class ResampleCopyFromReplication;
class ResamplePostCopy;
template <typename ResType> class Resample;
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
template <std::size_t, typename, typename ID = CLDefault> class StateCL;
template <typename, typename B = NullType> class InitializeCL;
template <typename, typename B = NullType> class MoveCL;
template <typename, typename B = NullType> class MonitorEvalCL;
template <typename, typename B = NullType> class PathEvalCL;

// MPI
struct MPIDefault;
template <typename> class MPICommunicator;
template <typename, typename ID = MPIDefault> class StateMPI;
template <typename> class WeightSetMPI;
class NormalizingConstant;

// SMP State
template <std::size_t> struct Position {};
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

// SMP Implementations
VSMC_DEFINE_SMP_FORWARD(SEQ)
VSMC_DEFINE_SMP_FORWARD(CILK)
VSMC_DEFINE_SMP_FORWARD(GCD)
VSMC_DEFINE_SMP_FORWARD(OMP)
VSMC_DEFINE_SMP_FORWARD(PPL)
VSMC_DEFINE_SMP_FORWARD(STD)
VSMC_DEFINE_SMP_FORWARD(TBB)

} // namesapce vsmc

/// \endcond HIDDEN_SYMBOLS

#endif // VSMC_INTERNAL_FORWARD_HPP
