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
#include <vSMC/internal/version.hpp>
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

class StateBaseTrait
{
    public :

    virtual ~StateBaseTrait () {}
};

class InitializeSeqTrait
{
    public :

    virtual ~InitializeSeqTrait () {}
};

class MoveSeqTrait
{
    public :

    virtual ~MoveSeqTrait () {}
};

class MonitorSeqTrait
{
    public :

    virtual ~MonitorSeqTrait () {}
};

class PathSeqTrait
{
    public :

    virtual ~PathSeqTrait () {}
};

class InitializeTBBTrait
{
    public :

    virtual ~InitializeTBBTrait () {}
};

class MoveTBBTrait
{
    public :

    virtual ~MoveTBBTrait () {}
};

class MonitorTBBTrait
{
    public :

    virtual ~MonitorTBBTrait () {}
};

class PathTBBTrait
{
    public :

    virtual ~PathTBBTrait () {}
};

class StateCLTrait
{
    public :

    virtual ~StateCLTrait () {}

    virtual void pre_resampling () = 0;
    virtual void post_resampling () = 0;
};

class InitializeCLTrait
{
    public :

    virtual ~InitializeCLTrait () {}
};

class MoveCLTrait
{
    public :

    virtual ~MoveCLTrait () {}
};

class MonitorCLTrait
{
    public :

    virtual ~MonitorCLTrait () {}
};

class PathCLTrait
{
    public :

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
