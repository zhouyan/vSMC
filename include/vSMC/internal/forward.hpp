#ifndef V_SMC_INTERNAL_FORWARD_HPP
#define V_SMC_INTERNAL_FORWARD_HPP

namespace vSMC {

template <typename T> class Sampler;
template <typename T> class Particle;
template <typename T> class Monitor;
template <typename T> class Path;

template <typename T> class SingleParticle;
template <typename T> class ConstSingleParticle;
template <unsigned Dim, typename T> class StateBase;

template <typename T> class InitializeSeq;
template <typename T> class MoveSeq;
template <typename T, unsigned Dim> class MonitorSeq;
template <typename T> class PathSeq;

template <typename T> class InitializeTBB;
template <typename T> class MoveTBB;
template <typename T, unsigned Dim> class MonitorTBB;
template <typename T> class PathTBB;

template <unsigned Dim, typename T> class StateCL;
template <typename T> class InitializeCL;
template <typename T> class MoveCL;
template <typename T, unsigned Dim> class MonitorCL;
template <typename T> class PathCL;

} // namesapce vSMC

#endif // V_SMC_INTERNAL_FORWARD_HPP
