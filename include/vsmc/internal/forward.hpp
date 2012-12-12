#ifndef VSMC_INTERNAL_FORWARD_HPP
#define VSMC_INTERNAL_FORWARD_HPP

namespace vsmc {

// Placeholders
class VBase;
class CBase;
class NullType;

// Utilities
class Seed;
template <typename T = NullType> class DDOT;
template <typename T = NullType> class DGEMV;

namespace thread {

class ThreadGuard;
class ThreadManager;
template <typename> class BlockedRange;

template <typename S, typename W>
void parallel_for (const BlockedRange<S> &, const W &);

template <typename S, typename W, typename R>
void parallel_sum (const BlockedRange<S> &, const W &, R &);

} // namespace vsmc::thread

// Core module
class RngSetSeq;
class RngSetPrl;
template <typename> class WeightSetBase;
template <typename> class Sampler;
template <typename> class Particle;
template <typename> class Monitor;
template <typename> class Path;
template <typename> class SingleParticle;
template <typename> class ConstSingleParticle;
template <typename, template <typename> class> class ParticleIterator;

// SMP
template <unsigned, typename> class StateBase;
template <typename, typename> class InitializeBase;
template <typename, typename> class MoveBase;
template <typename, typename> class MonitorEvalBase;
template <typename, typename> class PathEvalBase;

#if VSMC_HAS_CXX11_ALIAS_TEMPLATES
template <unsigned Dim, typename T> using StateSEQ = StateBase<Dim, T>;
#else
template <unsigned, typename> class StateSEQ;
#endif
template <typename, typename D = VBase> class InitializeSEQ;
template <typename, typename D = VBase> class MoveSEQ;
template <typename, typename D = VBase> class MonitorEvalSEQ;
template <typename, typename D = VBase> class PathEvalSEQ;

template <unsigned, typename> class StateSTD;
template <typename, typename D = VBase> class InitializeSTD;
template <typename, typename D = VBase> class MoveSTD;
template <typename, typename D = VBase> class MonitorEvalSTD;
template <typename, typename D = VBase> class PathEvalSTD;

template <unsigned, typename> class StateOMP;
template <typename, typename D = VBase> class InitializeOMP;
template <typename, typename D = VBase> class MoveOMP;
template <typename, typename D = VBase> class MonitorEvalOMP;
template <typename, typename D = VBase> class PathEvalOMP;

template <unsigned, typename> class StateTBB;
template <typename, typename D = VBase> class InitializeTBB;
template <typename, typename D = VBase> class MoveTBB;
template <typename, typename D = VBase> class MonitorEvalTBB;
template <typename, typename D = VBase> class PathEvalTBB;

template <unsigned, typename> class StateCILK;
template <typename, typename D = VBase> class InitializeCILK;
template <typename, typename D = VBase> class MoveCILK;
template <typename, typename D = VBase> class MonitorEvalCILK;
template <typename, typename D = VBase> class PathEvalCILK;

// OCL
template <unsigned, typename> class StateCL;
template <typename> class InitializeCL;
template <typename> class MoveCL;
template <typename> class MonitorEvalCL;
template <typename> class PathEvalCL;

template <typename T, template <typename, typename> class I = InitializeSEQ,
         typename B = CBase> class InitializeAdapter;
template <typename, template <typename, typename> class I = MoveSEQ,
         typename B = CBase> class MoveAdapter;
template <typename, template <typename, typename> class I = MonitorEvalSEQ,
         typename B = CBase> class MonitorEvalAdapter;
template <typename, template <typename, typename> class I = PathEvalSEQ,
         typename B = CBase> class PathEvalAdapter;

} // namesapce vsmc

#endif // VSMC_INTERNAL_FORWARD_HPP
