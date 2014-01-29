#ifndef VSMC_INTERNAL_ASSERT_HPP
#define VSMC_INTERNAL_ASSERT_HPP

#include <vsmc/internal/config.hpp>

#include <cassert>
#include <cstdio>
#include <stdexcept>

// Runtime assertion

#if VSMC_RUNTIME_ASSERT_AS_EXCEPTION
#define VSMC_RUNTIME_ASSERT(cond, msg)                                        \
{                                                                             \
    if (!(cond)) {                                                            \
        throw vsmc::RuntimeAssert(msg);                                       \
    };                                                                        \
}
#elif defined(NDEBUG) // No Debug
#define VSMC_RUNTIME_ASSERT(cond, msg)
#else // Runtime assertion
#define VSMC_RUNTIME_ASSERT(cond, msg)                                        \
{                                                                             \
    if (!(cond)) {                                                            \
        std::fprintf(stderr,                                                  \
                "vSMC runtime assertion failed; File: %s; Line: %d\n%s\n",    \
                __FILE__, __LINE__, msg);                                     \
    };                                                                        \
    assert(cond);                                                             \
}
#endif // VSMC_RUNTIME_ASSERT_AS_EXCEPTION

// Static assertion

#if VSMC_HAS_CXX11_STATIC_ASSERT
#define VSMC_STATIC_ASSERT(cond, msg) static_assert(cond, #msg)
#else // VSMC_HAS_CXX11_STATIC_ASSERT
#ifdef _MSC_VER
#define VSMC_STATIC_ASSERT(cond, msg) \
    {vsmc::StaticAssert<bool(cond)>::msg;}
#else // _MSC_VER
#define VSMC_STATIC_ASSERT(cond, msg) \
    if (vsmc::StaticAssert<bool(cond)>::msg) {};
#endif // _MSC_VER
#endif // VSMC_HAS_CXX11_STATIC_ASSERT

namespace vsmc {

class RuntimeAssert : public std::runtime_error
{
    public :

    RuntimeAssert (const std::string &msg) : std::runtime_error(msg) {}
}; // class RuntimeAssert

template <bool> class StaticAssert {};

template <>
class StaticAssert<true>
{
    public :

    enum {
        USE_METHOD_resize_dim_WITH_A_FIXED_SIZE_StateMatrix_OBJECT,
        USE_METHOD_resize_state_WITH_A_FIXED_SIZE_StateCL_OBJECT,

        USE_InitializeCL_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL,
        USE_MonitorEvalCL_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL,
        USE_MoveCL_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL,
        USE_PathEvalCL_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL,
        USE_StateCL_WITH_A_FP_TYPE_OTHER_THAN_cl_float_AND_cl_double,

        USE_NIntegrateNewtonCotes_WITH_A_DEGREE_LARGER_THAN_max_degree
    };
}; // class StaticAssert

} // namespace vsmc

// Static assertion macros

#define VSMC_STATIC_ASSERT_DYNAMIC_DIM_RESIZE(Dim)                           \
    VSMC_STATIC_ASSERT((Dim == vsmc::Dynamic),                               \
            USE_METHOD_resize_dim_WITH_A_FIXED_SIZE_StateMatrix_OBJECT)

#define VSMC_STATIC_ASSERT_DYNAMIC_STATE_SIZE_RESIZE(Dim)                    \
    VSMC_STATIC_ASSERT((Dim == vsmc::Dynamic),                               \
            USE_METHOD_resize_state_WITH_A_FIXED_SIZE_StateCL_OBJECT)

#define VSMC_STATIC_ASSERT_NINTEGRATE_NEWTON_COTES_DEGREE(degree)            \
    VSMC_STATIC_ASSERT((degree >= 1 && degree <= max_degree_),               \
            USE_NIntegrateNewtonCotes_WITH_A_DEGREE_LARGER_THAN_max_degree)

#define VSMC_STATIC_ASSERT_NO_IMPL(member)                                   \
    VSMC_STATIC_ASSERT((vsmc::cxx11::is_same<Derived, NullType>::value),     \
            NO_IMPLEMENTATION_OF_##member##_FOUND)

#define VSMC_STATIC_ASSERT_STATE_CL_TYPE(derived, user)                      \
    VSMC_STATIC_ASSERT((vsmc::traits::IsDerivedFromStateCL<derived>::value), \
            USE_##user##_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL)

#define VSMC_STATIC_ASSERT_STATE_CL_FP_TYPE(type)                            \
    VSMC_STATIC_ASSERT((vsmc::cxx11::is_same<type, cl_float>::value          \
                || vsmc::cxx11::is_same<type, cl_double>::value),            \
            USE_StateCL_WITH_A_FP_TYPE_OTHER_THAN_cl_float_AND_cl_double)

#endif // VSMC_INTERNAL_ASSERT_HPP
