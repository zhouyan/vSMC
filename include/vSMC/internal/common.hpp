#ifndef V_SMC_INTERNAL_COMMON_HPP
#define V_SMC_INTERNAL_COMMON_HPP

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif // __STDC_CONSTANT_MACROS

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <utility>
#include <map>
#include <set>
#include <deque>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>

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
#include <vSMC/internal/type_traits.hpp>
#include <vSMC/rng/random.hpp>

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

#ifdef V_SMC_USE_TBB
template <typename T> class InitializeTBB;
template <typename T> class MoveTBB;
template <typename T, unsigned Dim = 1> class MonitorTBB;
template <typename T> class PathTBB;
#endif // V_SMC_USE_TBB

#ifdef V_SMC_USE_CL
template <unsigned Dim = 1, typename T = cl_float> class StateCL;
template <typename T> class InitializeCL;
template <typename T> class MoveCL;
template <typename T, unsigned Dim = 1> class MonitorCL;
template <typename T> class PathCL;
#endif // V_SMC_USE_CL

struct StateBaseTrait
{
    virtual ~StateBaseTrait () {}
};

struct InitializeSeqTrait
{
    virtual ~InitializeSeqTrait () {}
};

struct MoveSeqTrait
{
    virtual ~MoveSeqTrait () {}
};

struct MonitorSeqTrait
{
    virtual ~MonitorSeqTrait () {}
};

struct PathSeqTrait
{
    virtual ~PathSeqTrait () {}
};

struct InitializeTBBTrait
{
    virtual ~InitializeTBBTrait () {}
};

struct MoveTBBTrait
{
    virtual ~MoveTBBTrait () {}
};

struct MonitorTBBTrait
{
    virtual ~MonitorTBBTrait () {}
};

struct PathTBBTrait
{
    virtual ~PathTBBTrait () {}
};

struct StateCLTrait
{
    virtual ~StateCLTrait () {}

    virtual void pre_resampling () = 0;
    virtual void post_resampling () = 0;
};

struct InitializeCLTrait
{
    virtual ~InitializeCLTrait () {}
};

struct MoveCLTrait
{
    virtual ~MoveCLTrait () {}
};

struct MonitorCLTrait
{
    virtual ~MonitorCLTrait () {}
};

struct PathCLTrait
{
    virtual ~PathCLTrait () {}
};

/// \brief Resample scheme
/// \ingroup Core
enum ResampleScheme {
    MULTINOMIAL,         ///< Multinomial resampling
    RESIDUAL,            ///< Reisudal resampling
    STRATIFIED,          ///< Startified resampling
    SYSTEMATIC,          ///< Systematic resampling
    RESIDUAL_STRATIFIED, ///< Stratified resampling on the residuals
    RESIDUAL_SYSTEMATIC  ///< Systematic resampling on the residuals
}; // enum ResamleScheme

} // namespace vSMC

#endif // V_SMC_INTERNAL_COMMON_HPP
