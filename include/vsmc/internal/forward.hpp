#ifndef VSMC_INTERNAL_FORWARD_HPP
#define VSMC_INTERNAL_FORWARD_HPP

namespace vsmc {

namespace internal {

class VirtualBase {};

template <typename T, typename Derived> class InitializeBase;
template <typename T, typename Derived> class MoveBase;
template <typename T, typename Derived> class MonitorBase;
template <typename T, typename Derived> class PathBase;

} // namespace vsmc::internal

template <typename T> class Sampler;
template <typename T> class Particle;
template <typename T> class Monitor;
template <typename T> class Path;

template <unsigned Dim, typename T> class StateBase;
template <typename T> class SingleParticle;
template <typename T> class ConstSingleParticle;

template <unsigned Dim, typename T> class StateSeq;
template <typename T> class InitializeSeq;
template <typename T, typename Derived = internal::VirtualBase> class MoveSeq;
template <typename T, unsigned Dim> class MonitorSeq;
template <typename T> class PathSeq;

template <unsigned Dim, typename T, typename P = NullProfiler> class StateTBB;
template <typename T> class InitializeTBB;
template <typename T> class MoveTBB;
template <typename T, unsigned Dim> class MonitorTBB;
template <typename T> class PathTBB;

template <unsigned Dim, typename T, typename P = NullProfiler> class StateCL;
template <typename T> class InitializeCL;
template <typename T> class MoveCL;
template <typename T, unsigned Dim> class MonitorCL;
template <typename T> class PathCL;

namespace internal {

class ParallelTag {};

} } // namesapce vsmc

#endif // VSMC_INTERNAL_FORWARD_HPP
