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
#else
#define VSMC_NULLPTR 0
#endif

#ifdef NDEBUG
#define VSMC_RUNTIME_ASSERT(cond, message)
#else // NDEBUG
#define VSMC_RUNTIME_ASSERT(cond, message)                   \
{                                                            \
    if (!(cond)) {                                           \
        std::cerr                                            \
            << "vSMC runtime assertion failed:" << std::endl \
            << message << std::endl;                         \
    };                                                       \
    assert(cond);                                            \
}
#endif // NDEBUG

#ifdef _MSC_VER
#define VSMC_STATIC_ASSERT(cond, message) \
    {vsmc::StaticAssert<bool(cond)>::message;}
#else
#define VSMC_STATIC_ASSERT(cond, message) \
    if (vsmc::StaticAssert<bool(cond)>::message) {};
#endif

#define VSMC_DEFINE_TYPE_DISPATCH_TRAIT(OuterType, inner_type, default_type) \
namespace vsmc {                                                             \
                                                                             \
namespace internal {                                                         \
                                                                             \
template <typename T>                                                        \
class Has##OuterType##Impl                                                   \
{                                                                            \
    struct char2 {char c1; char c2;};                                        \
    template <typename U> static char test (typename U::inner_type *);       \
    template <typename U> static char2 test (...);                           \
                                                                             \
    public :                                                                 \
                                                                             \
    static const bool value =                                                \
        sizeof(test<T>(VSMC_NULLPTR)) == sizeof(char);                       \
};                                                                           \
                                                                             \
template <typename T>                                                        \
class Has##OuterType :                                                       \
    public integral_constant<bool, Has##OuterType##Impl<T>::value>           \
{};                                                                          \
                                                                             \
template <typename T, bool> struct OuterType##Dispatch;                      \
                                                                             \
template <typename T> struct OuterType##Dispatch<T, true>                    \
{typedef typename T::inner_type type;};                                      \
                                                                             \
template <typename T> struct OuterType##Dispatch<T, false>                   \
{typedef default_type type;};                                                \
                                                                             \
}                                                                            \
                                                                             \
template <typename T> struct OuterType##Trait                                \
{                                                                            \
    typedef typename internal::OuterType##Dispatch<T,                        \
        internal::Has##OuterType<T>::value>::type type;                      \
};                                                                           \
                                                                             \
}

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(SizeType, size_type, VSMC_SIZE_TYPE);

namespace vsmc {

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

        USE_InitializeOMP_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateOMP,
        USE_MoveOMP_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateOMP,
        USE_MonitorEvalOMP_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateOMP,
        USE_PathEvalOMP_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateOMP,

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

} // namespace vsmc

#endif // VSMC_INTERNAL_COMMON_HPP
