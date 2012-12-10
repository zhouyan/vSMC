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

#include <iterator>
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

// Avoid MSVC stupid behavior
#define VSMC_MINMAX_NO_EXPANSION

// Runtime assertion
#if VSMC_RUNTIME_ASSERT_AS_EXCEPTION
#define VSMC_RUNTIME_ASSERT(cond, msg)  \
{                                       \
    if (!(cond)) {                      \
        throw vsmc::RuntimeAssert(msg); \
    };                                  \
}
#elif defined(NDEBUG) // VSMC_RUNTIME_ASSERT_AS_EXCEPTION
#define VSMC_RUNTIME_ASSERT(cond, msg)
#else
#define VSMC_RUNTIME_ASSERT(cond, msg)                       \
{                                                            \
    if (!(cond)) {                                           \
        std::cerr                                            \
            << "vSMC runtime assertion failed:" << std::endl \
            << msg << std::endl;                             \
    };                                                       \
    assert(cond);                                            \
}
#endif // VSMC_RUNTIME_ASSERT_AS_EXCEPTION

// Static assertion
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

// Type dispatcher
#define VSMC_DEFINE_TYPE_DISPATCH_TRAIT(OuterType, InnerType, DefaultType)   \
namespace vsmc { namespace traits {                                          \
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
                                                                             \
template <typename T> struct OuterType##Trait                                \
{                                                                            \
    enum {value = traits::Has##OuterType<T>::value};                         \
    typedef typename traits::OuterType##Dispatch<T, value>::type type;       \
};                                                                           \
                                                                             \
} }

#define VSMC_DEFINE_MEMBER_FUNCTION_CHECKER(OuterMF, InnerMF, RT, Args)      \
namespace vsmc { namespace traits {                                          \
                                                                             \
template <typename T>                                                        \
struct Has##OuterMF##Impl                                                    \
{                                                                            \
    private :                                                                \
                                                                             \
    struct char2 {char c1; char c2;};                                        \
    template <typename U, RT (U::*) Args> struct sfinae_;                    \
    template <typename U> static char test (sfinae_<U, &U::InnerMF> *);      \
    template <typename U> static char2 test(...);                            \
                                                                             \
    public :                                                                 \
                                                                             \
    enum {value = sizeof(test<T>(VSMC_NULLPTR)) == sizeof(char)};            \
};                                                                           \
                                                                             \
template <typename T>                                                        \
struct Has##OuterMF :                                                        \
    public cxx11::integral_constant<bool, Has##OuterMF##Impl<T>::value>      \
{};                                                                          \
                                                                             \
} }

#define VSMC_STATIC_ASSERT_STATE_TYPE(base, derived, user)                   \
    VSMC_STATIC_ASSERT((vsmc::traits::IsBaseOfState<base, derived>::value),  \
            USE_##user##_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_##base)

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(SizeType, size_type, VSMC_SIZE_TYPE);

namespace vsmc {

enum {Dynamic};
enum MatrixOrder     {RowMajor = 101, ColMajor = 102};
enum MatrixTranspose {NoTrans = 111, Trans = 112, ConjTrans = 113};

class RuntimeAssert : public std::runtime_error
{
    public :

    RuntimeAssert (const std::string &msg) : std::runtime_error(msg) {}

    RuntimeAssert (const char *msg) : std::runtime_error(msg) {}
}; // class RuntimeAssert

template <bool> class StaticAssert;

template <>
class StaticAssert<true>
{
    public :

    enum {
        USE_METHOD_resize_dim_WITH_A_FIXED_SIZE_StateBase_OBJECT,

        USE_StateCL_WITH_A_STATE_TYPE_OTHER_THAN_cl_float_AND_cl_double,
        USE_METHOD_resize_dim_WITH_A_FIXED_SIZE_StateCL_OBJECT,
        USE_InitializeCL_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL,
        USE_MoveCL_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL,
        USE_MonitorEvalCL_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL,
        USE_PathEvalCL_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL,

        USE_InitializeAdapter_WITHOUT_AN_INITIAILIZE_IMPLEMENTATION,
        USE_MoveAdapter_WITHOUT_A_MOVE_IMPLEMENTATION,
        USE_MonitorEvalAdapter_WITHOUT_A_MONITOR_EVAL_IMPLEMENTATION,
        USE_PathEvalAdapter_WITHOUT_A_PATH_EVAL_IMPLEMENTATION
    };
}; // class StaticAssert

} // namespace vsmc

#endif // VSMC_INTERNAL_COMMON_HPP
