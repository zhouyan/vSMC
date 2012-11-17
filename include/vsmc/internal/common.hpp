#ifndef VSMC_INTERNAL_COMMON_HPP
#define VSMC_INTERNAL_COMMON_HPP

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include <algorithm>
#include <limits>
#include <numeric>

#include <deque>
#include <map>
#include <string>
#include <vector>

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <stdexcept>
#include <utility>

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/forward.hpp>

#include <vsmc/cxx11/functional.hpp>
#include <vsmc/cxx11/random.hpp>
#include <vsmc/cxx11/type_traits.hpp>

#include <vsmc/utility/seed.hpp>

#define VSMC_MINMAX_NO_EXPANSION

#if VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS
#define VSMC_EXPLICIT_OPERATOR explicit
#else
#define VSMC_EXPLICIT_OPERATOR
#endif

#ifdef NDEBUG
#define VSMC_RUNTIME_ASSERT(cond, msg)
#else // NDEBUG
#define VSMC_RUNTIME_ASSERT(cond, msg)                       \
{                                                            \
    if (!(cond)) {                                           \
        std::cerr                                            \
            << "vSMC runtime assertion failed:" << std::endl \
            << msg << std::endl;                             \
    };                                                       \
    assert(cond);                                            \
}
#endif // NDEBUG

#if VSMC_HAS_CXX11_STATIC_ASSERT

#define VSMC_STATIC_ASSERT(cond, msg) static_assert(cond, #msg)

#elif defined(BOOST_STATIC_ASSERT) // VSMC_HAS_CXX11_STATIC_ASSERT

#define VSMC_STATIC_ASSERT(cond, msg) BOOST_STATIC_ASSERT_MSG(cond, #msg)

#else // VSMC_HAS_CXX11_STATIC_ASSERT

#ifdef _MSC_VER
#define VSMC_STATIC_ASSERT(cond, msg) \
    {vsmc::StaticAssert<bool(cond)>::msg;}
#else // _MSC_VER
#define VSMC_STATIC_ASSERT(cond, msg) \
    if (vsmc::StaticAssert<bool(cond)>::msg) {};
#endif // _MSC_VER

#endif // VSMC_HAS_CXX11_STATIC_ASSERT

#define VSMC_DEFINE_TYPE_DISPATCH_TRAIT(OuterType, InnerType, DefaultType)   \
namespace vsmc {                                                             \
                                                                             \
namespace traits {                                                           \
                                                                             \
template <typename T>                                                        \
struct Has##OuterType##Impl                                                  \
{                                                                            \
    private :                                                                \
                                                                             \
    struct char2 {char c1; char c2;};                                        \
    template <typename U> static char test (typename U::InnerType *);        \
    template <typename U> static char2 test (...);                           \
                                                                             \
    public :                                                                 \
                                                                             \
    enum {value = sizeof(test<T>(VSMC_NULLPTR)) == sizeof(char)};            \
};                                                                           \
                                                                             \
template <typename T>                                                        \
struct Has##OuterType :                                                      \
    public cxx11::integral_constant <bool, Has##OuterType##Impl<T>::value>   \
{};                                                                          \
                                                                             \
template <typename T, bool> struct OuterType##Dispatch;                      \
                                                                             \
template <typename T> struct OuterType##Dispatch<T, true>                    \
{typedef typename T::InnerType type;};                                       \
                                                                             \
template <typename T> struct OuterType##Dispatch<T, false>                   \
{typedef DefaultType type;};                                                 \
                                                                             \
}                                                                            \
                                                                             \
template <typename T> struct OuterType##Trait                                \
{                                                                            \
    enum {value = traits::Has##OuterType<T>::value};                         \
    typedef typename traits::OuterType##Dispatch<T, value>::type type;       \
};                                                                           \
                                                                             \
}

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(SizeType, size_type, VSMC_SIZE_TYPE);

namespace vsmc {

enum {Dynamic};

enum MatrixOrder {ColumnMajor, RowMajor};

template <bool> class StaticAssert {};

template <>
class StaticAssert<true>
{
    public :

    enum {
        USE_InitializeSEQ_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateSEQ,
        USE_MoveSEQ_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateSEQ,
        USE_MonitorEvalSEQ_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateSEQ,
        USE_PathEvalSEQ_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateSEQ,

        USE_InitializeCILK_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCILK,
        USE_MoveCILK_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCILK,
        USE_MonitorEvalCILK_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCILK,
        USE_PathEvalCILK_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCILK,

        USE_InitializeOMP_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateOMP,
        USE_MoveOMP_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateOMP,
        USE_MonitorEvalOMP_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateOMP,
        USE_PathEvalOMP_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateOMP,

        USE_InitializeSTD_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateSTD,
        USE_MoveSTD_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateSTD,
        USE_MonitorEvalSTD_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateSTD,
        USE_PathEvalSTD_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateSTD,

        USE_InitializeTBB_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateTBB,
        USE_MoveTBB_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateTBB,
        USE_MonitorEvalTBB_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateTBB,
        USE_PathEvalTBB_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateTBB,

        USE_InitializeCL_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL,
        USE_MoveCL_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL,
        USE_MonitorEvalCL_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL,
        USE_PathEvalCL_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL,
        USE_StateCL_WITH_A_STATE_TYPE_OTHER_THAN_cl_float_AND_cl_double,

        USE_ConstSingleParticle_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateBase,
        USE_SingleParticle_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateBase,

        USE_METHOD_resize_dim_WITH_A_FIXED_SIZE_StateBase_OBJECT,
        USE_METHOD_resize_dim_WITH_A_FIXED_SIZE_StateCL_OBJECT
    };
};

} // namespace vsmc

#endif // VSMC_INTERNAL_COMMON_HPP
