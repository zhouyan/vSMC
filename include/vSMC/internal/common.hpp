#ifndef V_SMC_INTERNAL_COMMON_HPP
#define V_SMC_INTERNAL_COMMON_HPP

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif // __STDC_CONSTANT_MACROS

#include <vector>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <Eigen/Dense>
#include <vSMC/internal/version.hpp>
#include <vSMC/internal/config.hpp>
#include <vSMC/internal/function.hpp>
#include <vSMC/internal/random.hpp>

#ifndef V_SMC_INDEX_TYPE
#define V_SMC_INDEX_TYPE EIGEN_DEFAULT_DENSE_INDEX_TYPE 
#endif // V_SMC_INDEX_TYPE

namespace vSMC {

template <typename T> class Sampler;
template <typename T> class Particle;
template <typename T> class Monitor;
template <typename T> class Path;

template <typename T> class SingleParticle;
template <unsigned Dim = 1, typename T = double> class StateBase;

template <typename T> class InitializeSeq;
template <typename T> class MoveSeq;
template <typename T, unsigned Dim = 1> class MonitorSeq;
template <typename T> class PathSeq;

template <typename T> class InitializeTBB;
template <typename T> class MoveTBB;
template <typename T, unsigned Dim = 1> class MonitorTBB;
template <typename T> class PathTBB;

/// \brief Resample scheme
///
/// \li MULTINOMIAL Multinomial resampling
/// \li RESIDUAL Reisudal resampling
/// \li STRATIFIED Startified resampling
/// \li SYSTEMATIC Systematic resampling
/// \li RESIDUAL_STRATIFIED Stratified resampling on the residuals
/// \li RESIDUAL_SYSTEMATIC Systematic resampling on the residuals
enum ResampleScheme {
    MULTINOMIAL,
    RESIDUAL,
    STRATIFIED,
    SYSTEMATIC,
    RESIDUAL_STRATIFIED,
    RESIDUAL_SYSTEMATIC
};

/// \brief Type of weight returned by MoveSeq::move_state
///
/// \li No_ACTION The weight is discarded without further action
/// \li SET_WEIGHT The weight is set directly as the new weight
/// \li SET_LOG_WEIGHT The weight is set direclty as the new log weight
/// \li MUL_WEIGHT The weight is the incremental weight
/// \li ADD_LOG_WEIGHT The weight is the log of the incremental weight
enum WeightAction {
    NO_ACTION,
    SET_WEIGHT,
    SET_LOG_WEIGHT,
    MUL_WEIGHT,
    ADD_LOG_WEIGHT
};

} // namespace vSMC

#endif // V_SMC_INTERNAL_COMMON_HPP
