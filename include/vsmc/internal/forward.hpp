#ifndef VSMC_INTERNAL_FORWARD_HPP
#define VSMC_INTERNAL_FORWARD_HPP

/// \cond HIDDEN_SYMBOLS

#define VSMC_DEFINE_SMP_FORWARD(Name) \
namespace vsmc {                                                             \
    template <std::size_t, typename> class State##Name;                      \
    template <typename, typename D = VBase> class Initialize##Name;          \
    template <typename, typename D = VBase> class Move##Name;                \
    template <typename, typename D = VBase> class MonitorEval##Name;         \
    template <typename, typename D = VBase> class PathEval##Name;            \
}

namespace vsmc {

// Placeholders
class CBase;
class VBase;
class NullType;

// Utilities
class Seed;

namespace cxxblas {

class DDot;
class DGemv;
class ISUnivariate;
class ISMultivariate;

} // namespace vsmc::cxxblas

namespace gcd {

class DispatchQueue;

} // namespace vsmc::gcd

namespace thread {

template <typename> class BlockedRange;
class ThreadGuard;
class ThreadInfo;

} // namespace vsmc::thread

namespace opencl {

struct Default;
struct All;
struct CPU;
struct GPU;
struct Accelerator;

struct AMD;
struct Apple;
struct Intel;
struct NVIDIA;

struct AMDCPU;
struct AMDGPU;
struct AppleCPU;
struct AppleGPU;
struct IntelCPU;
struct IntelGPU;
struct NVIDIACPU;
struct NVIDIAGPU;

class LocalSize;

template <typename> class Manager;

} // namespace vsmc::opencl

// Core
class Seed;
class RngSetSeq;
class RngSetPrl;
template <typename ResType> class Resample;
template <typename> class WeightSet;
template <typename> class Sampler;
template <typename> class Particle;
template <typename> class Monitor;
template <typename> class Path;
template <typename> class SingleParticle;
template <typename> class ConstSingleParticle;
template <typename, template <typename> class> class ParticleIterator;

// OpenCL
template <std::size_t, typename, typename ID = VSMC_CL_DEFAULT_ID>
class StateCL;
template <typename> class InitializeCL;
template <typename> class MoveCL;
template <typename> class MonitorEvalCL;
template <typename> class PathEvalCL;

// SMP Base
template <std::size_t, typename> class StateBase;
template <typename, typename> class InitializeBase;
template <typename, typename> class MoveBase;
template <typename, typename> class MonitorEvalBase;
template <typename, typename> class PathEvalBase;

// SMP Adapter
template <typename, template <typename, typename> class, typename B = CBase>
         class InitializeAdapter;
template <typename, template <typename, typename> class, typename B = CBase>
         class MoveAdapter;
template <typename, template <typename, typename> class, typename B = CBase>
         class MonitorEvalAdapter;
template <typename, template <typename, typename> class, typename B = CBase>
         class PathEvalAdapter;

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
