#ifndef V_SMC_INTERNAL_COMMON_HPP
#define V_SMC_INTERNAL_COMMON_HPP

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif // __STDC_CONSTANT_MACROS

#include <deque>
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

template <typename Base> class InitializeBase;
template <typename Base> class MoveBase;
template <typename Base> class MonitorBase;
template <typename Base> class PathBase;

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

} // namespace vSMC

#endif // V_SMC_INTERNAL_COMMON_HPP
