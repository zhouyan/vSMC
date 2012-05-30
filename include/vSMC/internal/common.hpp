#ifndef V_SMC_INTERNAL_COMMON_HPP
#define V_SMC_INTERNAL_COMMON_HPP

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif // __STDC_CONSTANT_MACROS

#include <cassert>
#include <cmath>
#include <cstddef>
#include <deque>
#include <map>
#include <ostream>
#include <set>
#include <string>
#include <vector>

#ifdef __INTEL_COMPILER
// #pragma warning(push)
#pragma warning(disable:2196)
#pragma warning(disable:2536)
#endif // __INTEL_COMPILER

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wall"
#pragma clang diagnostic ignored "-Wunique-enum"
#endif // __clang__

#include <Eigen/Dense>

#ifdef __INTEL_COMPILER
// #pragma warning(pop)
#endif // __INTEL_COMPILER

#ifdef __clang__
#pragma clang diagnostic pop
#endif // __clang__

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
template <typename T> class ConstSingleParticle;
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
