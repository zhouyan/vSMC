#ifndef VSMC_INTERNAL_FORWARD_HPP
#define VSMC_INTERNAL_FORWARD_HPP

namespace vsmc {

namespace internal {

class VBase {};
class PreResamplingTag {};
class PostResamplingTag {};

template <typename, typename> class InitializeBase;
template <typename, typename> class MoveBase;
template <typename, typename> class MonitorBase;
template <typename, typename> class PathBase;

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
template <typename, typename I = internal::VBase> class InitializeSeq;
template <typename, typename I = internal::VBase> class MoveSeq;
template <typename, unsigned, typename I = internal::VBase> class MonitorSeq;
template <typename, typename I = internal::VBase> class PathSeq;

template <unsigned, typename, typename P = NullTimer> class StateTBB;
template <typename, typename I = internal::VBase> class InitializeTBB;
template <typename, typename I = internal::VBase> class MoveTBB;
template <typename, unsigned, typename I = internal::VBase> class MonitorTBB;
template <typename, typename I = internal::VBase> class PathTBB;

template <unsigned, typename, typename P = NullTimer> class StateCL;
template <typename> class InitializeCL;
template <typename> class MoveCL;
template <typename, unsigned> class MonitorCL;
template <typename> class PathCL;

} // namesapce vsmc

#endif // VSMC_INTERNAL_FORWARD_HPP
