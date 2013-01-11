#ifndef VSMC_INTERNAL_FORWARD_HPP
#define VSMC_INTERNAL_FORWARD_HPP

namespace vsmc {

// Placeholders
class CBase;
class VBase;
class NullType;

// Utilities
class Seed;

namespace cxxblas {

template <typename T = NullType> class DDot;
template <typename T = NullType> class DGemv;

} // namespace vsmc::cxxblas

namespace thread {

class ThreadGuard;
class ThreadInfo;
template <typename> class BlockedRange;

template <typename S, typename W>
void parallel_for (const BlockedRange<S> &, const W &);

template <typename S, typename W, typename R>
void parallel_sum (const BlockedRange<S> &, const W &, R &);

} // namespace vsmc::thread

namespace opencl {

struct Default;
struct CPU;
struct GPU;
struct Accelerator;
template <typename> class CLManager;

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

// SMP

// Base
template <unsigned, typename> class StateBase;
template <typename, typename> class InitializeBase;
template <typename, typename> class MoveBase;
template <typename, typename> class MonitorEvalBase;
template <typename, typename> class PathEvalBase;

// Adapter
template <typename, template <typename, typename> class, typename B = CBase>
         class InitializeAdapter;
template <typename, template <typename, typename> class, typename B = CBase>
         class MoveAdapter;
template <typename, template <typename, typename> class, typename B = CBase>
         class MonitorEvalAdapter;
template <typename, template <typename, typename> class, typename B = CBase>
         class PathEvalAdapter;

// Sequential
#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <unsigned Dim, typename T> using StateSEQ = StateBase<Dim, T>;
#else
template <unsigned, typename> class StateSEQ;
#endif
template <typename, typename D = VBase> class InitializeSEQ;
template <typename, typename D = VBase> class MoveSEQ;
template <typename, typename D = VBase> class MonitorEvalSEQ;
template <typename, typename D = VBase> class PathEvalSEQ;

// Intel Cilk Plus
template <unsigned, typename> class StateCILK;
template <typename, typename D = VBase> class InitializeCILK;
template <typename, typename D = VBase> class MoveCILK;
template <typename, typename D = VBase> class MonitorEvalCILK;
template <typename, typename D = VBase> class PathEvalCILK;

// OpenMP
template <unsigned, typename> class StateOMP;
template <typename, typename D = VBase> class InitializeOMP;
template <typename, typename D = VBase> class MoveOMP;
template <typename, typename D = VBase> class MonitorEvalOMP;
template <typename, typename D = VBase> class PathEvalOMP;

// C++11 <thread>
template <unsigned, typename> class StateSTD;
template <typename, typename D = VBase> class InitializeSTD;
template <typename, typename D = VBase> class MoveSTD;
template <typename, typename D = VBase> class MonitorEvalSTD;
template <typename, typename D = VBase> class PathEvalSTD;

// Intel TBB
template <unsigned, typename> class StateTBB;
template <typename, typename D = VBase> class InitializeTBB;
template <typename, typename D = VBase> class MoveTBB;
template <typename, typename D = VBase> class MonitorEvalTBB;
template <typename, typename D = VBase> class PathEvalTBB;

// OpenCL
template <unsigned, typename, typename ID = VSMC_CL_DEFAULT_ID> class StateCL;
template <typename> class InitializeCL;
template <typename> class MoveCL;
template <typename> class MonitorEvalCL;
template <typename> class PathEvalCL;

} // namesapce vsmc

#endif // VSMC_INTERNAL_FORWARD_HPP
