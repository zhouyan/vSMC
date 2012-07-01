#ifndef VSMC_INTERNAL_COMMON_HPP
#define VSMC_INTERNAL_COMMON_HPP

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif // __STDC_CONSTANT_MACROS

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <utility>
#include <limits>
#include <map>
#include <set>
#include <deque>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>

#include <Eigen/Dense>

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/version.hpp>
#include <vsmc/internal/functional.hpp>
#include <vsmc/internal/type_traits.hpp>
#include <vsmc/internal/forward.hpp>

#if VSMC_HAS_CXX11_NULLPTR && VSMC_HAS_CXX11LIB_FUNCTIONAL
#define VSMC_NULLPTR nullptr
#else // VSMC_HAS_CXX11_NULLPTR
#define VSMC_NULLPTR NULL
#endif

#ifdef NDEBUG
#define VSMC_RUNTIME_ASSERT(cond, message)
#else // NDEBUG
#define VSMC_RUNTIME_ASSERT(cond, message) \
{ \
    if (!(cond)) \
        std::cerr \
            << "vSMC runtime assertion failed:" << std::endl \
            << message << std::endl; \
    else \
        ; \
    assert(cond); \
}
#endif // NDEBUG

namespace vsmc { namespace internal {

template <bool> class StaticAssert {};

template <>
class StaticAssert<true>
{
    public :

    enum {
        USE_InitializeSeq_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateSeq,
        USE_MoveSeq_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateSeq,
        USE_MonitorEvalSeq_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateSeq,
        USE_PathEvalSeq_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateSeq,

        USE_InitializeTBB_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateTBB,
        USE_MoveTBB_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateTBB,
        USE_MonitorEvalTBB_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateTBB,
        USE_PathEvalTBB_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateTBB,

        USE_InitializeCL_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL,
        USE_MoveCL_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL,
        USE_MonitorEvalCL_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL,
        USE_PathEvalCL_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL,

        USE_SingleParticle_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateBase,
        USE_ConstSingleParticle_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateBase
    };
};

} // namespace vsmc::internal

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

} // namespace vsmc

#ifdef _MSC_VER
#define VSMC_STATIC_ASSERT(cond, message) \
    {vsmc::internal::StaticAssert<bool(cond)>::message;}
#else
#define VSMC_STATIC_ASSERT(cond, message) \
    if (vsmc::internal::StaticAssert<bool(cond)>::message) {};
#endif

// IMPORTANT: this macro shall only be inserted at global scope
#define VSMC_DEFINE_TYPE_DISPATCH_TRAIT(outer_type, inner_type, default_type) \
namespace vsmc { namespace internal { \
\
template <typename T> \
class Has##outer_type \
{ \
    struct char2 {char c1; char c2;}; \
    template <typename U> static char test (typename U::inner_type *); \
    template <typename U> static char2 test (...); \
    public : \
    static const bool value = sizeof(test<T>(VSMC_NULLPTR)) == sizeof(char); \
}; \
\
template <typename T, bool> struct outer_type##Dispatch; \
template <typename T> struct outer_type##Dispatch<T, true> \
{typedef typename T::inner_type type; }; \
template <typename T> struct outer_type##Dispatch<T, false> \
{typedef default_type type;}; } \
\
template <typename T> struct outer_type##Trait \
{typedef typename internal::outer_type##Dispatch<T, \
    internal::Has##outer_type<T>::value>::type type;}; }

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(SizeType, size_type, VSMC_SIZE_TYPE);

#endif // VSMC_INTERNAL_COMMON_HPP
