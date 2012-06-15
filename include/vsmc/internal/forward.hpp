#ifndef VSMC_INTERNAL_FORWARD_HPP
#define VSMC_INTERNAL_FORWARD_HPP

namespace vsmc {

namespace internal {

class VirtualDerived {};
class PreResamplingTag {};
class PostResamplingTag {};

template <typename, typename> class InitializeBase;
template <typename, typename> class MoveBase;
template <typename, unsigned, typename> class MonitorEvalBase;
template <typename, typename> class PathEvalBase;

} // namespace vsmc::internal

class NullTimer;

template <typename> class Sampler;
template <typename> class Particle;
template <typename> class Monitor;
template <typename> class Path;

template <unsigned, typename> class StateBase;
template <typename> class SingleParticle;
template <typename> class ConstSingleParticle;

template <unsigned, typename, typename P = NullTimer> class StateSeq;
template <typename, typename D = internal::VirtualDerived> class InitializeSeq;
template <typename, typename D = internal::VirtualDerived> class MoveSeq;
template <typename, unsigned, typename D = internal::VirtualDerived>
class MonitorEvalSeq;
template <typename, typename D = internal::VirtualDerived> class PathEvalSeq;

template <unsigned, typename, typename P = NullTimer> class StateTBB;
template <typename, typename D = internal::VirtualDerived> class InitializeTBB;
template <typename, typename D = internal::VirtualDerived> class MoveTBB;
template <typename, unsigned, typename D = internal::VirtualDerived>
class MonitorEvalTBB;
template <typename, typename D = internal::VirtualDerived> class PathEvalTBB;

template <unsigned, typename, typename P = NullTimer> class StateCL;
template <typename> class InitializeCL;
template <typename> class MoveCL;
template <typename, unsigned> class MonitorEvalCL;
template <typename> class PathEvalCL;

} // namesapce vsmc

#endif // VSMC_INTERNAL_FORWARD_HPP
