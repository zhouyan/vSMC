#ifndef VSMC_INTERNAL_FORWARD_HPP
#define VSMC_INTERNAL_FORWARD_HPP

namespace vsmc {

template <typename T> class Sampler;
template <typename T> class Particle;
template <typename T> class Monitor;
template <typename T> class Path;

template <unsigned Dim, typename T> class StateBase;
template <typename T> class SingleParticle;
template <typename T> class ConstSingleParticle;

template <unsigned Dim, typename T> class StateSeq;
template <typename T> class InitializeSeq;
template <typename T> class MoveSeq;
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

} // namesapce vsmc

#endif // VSMC_INTERNAL_FORWARD_HPP
