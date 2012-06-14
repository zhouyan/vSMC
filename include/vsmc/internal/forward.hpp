#ifndef VSMC_INTERNAL_FORWARD_HPP
#define VSMC_INTERNAL_FORWARD_HPP

namespace vsmc {

namespace internal {

class VBase {};

template <typename, typename> class InitializeBase;
template <typename, typename> class MoveBase;
template <typename, typename> class MonitorBase;
template <typename, typename> class PathBase;

} // namespace vsmc::internal

template <typename> class Sampler;
template <typename> class Particle;
template <typename> class Monitor;
template <typename> class Path;

template <unsigned, typename> class StateBase;
template <typename> class SingleParticle;
template <typename> class ConstSingleParticle;

template <unsigned, typename> class StateSeq;
template <typename, typename D = internal::VBase> class InitializeSeq;
template <typename, typename D = internal::VBase> class MoveSeq;
template <typename, unsigned, typename D = internal::VBase> class MonitorSeq;
template <typename, typename D = internal::VBase> class PathSeq;

template <unsigned, typename, typename P = NullProfiler> class StateTBB;
template <typename, typename D = internal::VBase> class InitializeTBB;
template <typename, typename D = internal::VBase> class MoveTBB;
template <typename, unsigned, typename D = internal::VBase> class MonitorTBB;
template <typename, typename D = internal::VBase> class PathTBB;

template <unsigned, typename, typename P = NullProfiler> class StateCL;
template <typename> class InitializeCL;
template <typename> class MoveCL;
template <typename, unsigned> class MonitorCL;
template <typename> class PathCL;

namespace internal {

class ParallelTag {};

} } // namesapce vsmc

#endif // VSMC_INTERNAL_FORWARD_HPP
