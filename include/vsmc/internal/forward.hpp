#ifndef VSMC_INTERNAL_FORWARD_HPP
#define VSMC_INTERNAL_FORWARD_HPP

namespace vsmc {

namespace internal {

class WeightBase;
template <typename> class Weight;
template <typename, typename> class InitializeBase;
template <typename, typename> class MoveBase;
template <typename, typename> class MonitorEvalBase;
template <typename, typename> class PathEvalBase;

} // namespace vsmc::internal

class NullTimer;

class VBase {};

template <typename> class Sampler;
template <typename> class Particle;
template <typename> class Monitor;
template <typename> class Path;

template <unsigned, typename, typename> class StateBase;
template <typename> class SingleParticle;
template <typename> class ConstSingleParticle;

template <unsigned, typename, typename T = NullTimer> class StateSeq;
template <typename, typename D = VBase> class InitializeSeq;
template <typename, typename D = VBase> class MoveSeq;
template <typename, typename D = VBase> class MonitorEvalSeq;
template <typename, typename D = VBase> class PathEvalSeq;

template <unsigned, typename, typename T = NullTimer> class StateOMP;
template <typename, typename D = VBase> class InitializeOMP;
template <typename, typename D = VBase> class MoveOMP;
template <typename, typename D = VBase> class MonitorEvalOMP;
template <typename, typename D = VBase> class PathEvalOMP;

template <unsigned, typename, typename T = NullTimer> class StateTBB;
template <typename, typename D = VBase> class InitializeTBB;
template <typename, typename D = VBase> class MoveTBB;
template <typename, typename D = VBase> class MonitorEvalTBB;
template <typename, typename D = VBase> class PathEvalTBB;

template <unsigned, typename, typename P = NullTimer> class StateCL;
template <typename> class InitializeCL;
template <typename> class MoveCL;
template <typename> class MonitorEvalCL;
template <typename> class PathEvalCL;

template <unsigned> struct BayesianParamID;
template <typename, template <unsigned, typename, typename> class,
         typename T = NullTimer>
class BayesianState;
template <typename T, template <typename, typename> class, typename>
class BayesianMove;

} // namesapce vsmc

#endif // VSMC_INTERNAL_FORWARD_HPP
