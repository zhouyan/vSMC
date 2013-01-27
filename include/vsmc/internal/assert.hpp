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
#elif defined(VSMC_NDEBUG) // VSMC_RUNTIME_ASSERT_AS_EXCEPTION
#define VSMC_RUNTIME_ASSERT(cond, msg)
#else
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

    RuntimeAssert (const char *msg) : std::runtime_error(msg) {}
}; // class RuntimeAssert

template <bool> class StaticAssert {};

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

        USE_InitializeAdapter_WITHOUT_A_INITIAILIZE_IMPLEMENTATION,
        USE_MoveAdapter_WITHOUT_A_MOVE_IMPLEMENTATION,
        USE_MonitorEvalAdapter_WITHOUT_A_MONITOR_EVAL_IMPLEMENTATION,
        USE_PathEvalAdapter_WITHOUT_A_PATH_EVAL_IMPLEMENTATION,

        USE_NumericNewtonCotes_WITH_A_DEGREE_LARGER_THAN_4
    };
}; // class StaticAssert

} // namespace vsmc

// Runtime assertion macros

#define VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(func)                            \
    VSMC_RUNTIME_ASSERT((setup()),                                            \
            ("**vsmc::Manager::"#func"** CAN ONLY BE CALLED AFTER TRUE "      \
             "**vsmc::Manager::setup**"));

#define VSMC_RUNTIME_ASSERT_CONST_SINGLE_PARTICLE_VALID                       \
    VSMC_RUNTIME_ASSERT(particle_ptr_,                                        \
            ("A **ConstSingleParticle** object "                              \
             "is contructed with 0 **Particle** pointer"));                   \
VSMC_RUNTIME_ASSERT((id_ >= 0 && id_ <= particle_ptr_->size()),               \
        ("A **ConstSignleParticle** object "                                  \
         "is contructed with an out of range id"));

#define VSMC_RUNTIME_ASSERT_DERIVED_BASE(basename)                            \
    VSMC_RUNTIME_ASSERT((dynamic_cast<Derived *>(this)),                      \
            ("DERIVED FROM " #basename                                        \
             " WITH INCORRECT **Derived** TEMPLATE PARAMTER"));

#define VSMC_RUNTIME_ASSERT_FUNCTOR(func, name, caller)                       \
    VSMC_RUNTIME_ASSERT(bool(func), "**"#caller"** INVALID "#name" OBJECT")   \

#define VSMC_RUNTIME_ASSERT_ID_NUMBER(func)                                   \
    VSMC_RUNTIME_ASSERT((id >= 0 && id < this->dim()),                        \
            ("**"#func"** INVALID ITERATION NUMBER ARGUMENT"))

#define VSMC_RUNTIME_ASSERT_INVALID_MEMCPY_IN(diff, size, func)               \
    VSMC_RUNTIME_ASSERT((std::abs(diff) > static_cast<std::ptrdiff_t>(size)), \
            ("THE DESTINATION OF **"#func"** OVERLAPPING WITH THE SOURCE"))

#define VSMC_RUNTIME_ASSERT_INVALID_MEMCPY_OUT(diff, size, func)              \
    VSMC_RUNTIME_ASSERT((std::abs(diff) > static_cast<std::ptrdiff_t>(size)), \
            ("THE SOURCE OF **"#func"** OVERLAPPING WITH THE DESTINATION"))

#define VSMC_RUNTIME_ASSERT_ITERATION_NUMBER(func)                            \
    VSMC_RUNTIME_ASSERT((iter >= 0 && iter < this->iter_size()),              \
            ("**"#func"** INVALID ITERATION NUMBER ARGUMENT"))

#define VSMC_RUNTIME_ASSERT_MATRIX_ORDER(order, func)                         \
    VSMC_RUNTIME_ASSERT((order == vsmc::RowMajor || order == vsmc::ColMajor), \
            ("**"#func"** INVALID MATRIX ORDER"))

#define VSMC_RUNTIME_ASSERT_MONITOR_NAME(iter, map, func)                     \
    VSMC_RUNTIME_ASSERT((iter != map.end()),                                  \
            ("**"#func"** INVALID MONITOR NAME"))

#define VSMC_RUNTIME_ASSERT_PARTICLE_ITERATOR_BINARY_OP                       \
    VSMC_RUNTIME_ASSERT((iter1->particle_ptr() == iter2->particle_ptr()),     \
            ("BINARY OPERATION ON TWO **ParticleIterator** BELONGING TO "     \
            "TWO DIFFERNT **Particle** OBJECT"))

#define VSMC_RUNTIME_ASSERT_RANGE(begin, end, func)                           \
    VSMC_RUNTIME_ASSERT((begin < end), ("**"#func"** INVALID RANGE"))

#define VSMC_RUNTIME_ASSERT_SINGLE_PARTICLE_VALID                             \
    VSMC_RUNTIME_ASSERT(particle_ptr_,                                        \
            ("A **SingleParticle** OBJECT "                                   \
             "IS CONTRUCTED WITH 0 **Particle** POINTER"));                   \
VSMC_RUNTIME_ASSERT((id_ >= 0 && id_ <= particle_ptr_->size()),               \
        ("A **SignleParticle** OBJECT "                                       \
         "IS CONTRUCTED WITH AN OUT OF RANGE ID"));

#define VSMC_RUNTIME_ASSERT_STATE_CL_BUILD(func)                              \
    VSMC_RUNTIME_ASSERT((build()),                                            \
            ("**StateCL::"#func"** CAN ONLY BE CALLED AFTER true "            \
             "**StateCL::build**"));

#define VSMC_RUNTIME_ASSERT_STATE_COPY_SIZE_MISMATCH(name)                    \
    VSMC_RUNTIME_ASSERT((N == this->size()),                                  \
            ("**State"#name"::copy** SIZE MISMATCH"))

// Static assertion macros

#define VSMC_STATIC_ASSERT_ADAPTER_IMPL(name, NAME)                           \
    VSMC_STATIC_ASSERT(vsmc::traits::Is##name##Impl<Impl>::value,             \
            USE_##name##Adapter_WITHOUT_A_##NAME##_IMPLEMENTATION)            \

#define VSMC_STATIC_ASSERT_DYNAMIC_DIM_RESIZE(name)                           \
    VSMC_STATIC_ASSERT((Dim == vsmc::Dynamic),                                \
            USE_METHOD_resize_dim_WITH_A_FIXED_SIZE_State##name##_OBJECT)

#define VSMC_STATIC_ASSERT_NUMERIC_NEWTON_COTES_DEGREE(degree)                \
    VSMC_STATIC_ASSERT((degree <= 4),                                         \
            USE_NumericNewtonCotes_WITH_A_DEGREE_LARGER_THAN_4)

#define VSMC_STATIC_ASSERT_NO_IMPL(member)                                    \
    VSMC_STATIC_ASSERT((cxx11::is_same<Derived, NullType>::value),            \
            NO_IMPLEMENTATION_OF_##member##_FOUND)

#define VSMC_STATIC_ASSERT_STATE_CL_TYPE(derived, user)                       \
    VSMC_STATIC_ASSERT((vsmc::traits::IsBaseOfStateCL<derived>::value),       \
            USE_##user##_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL)

#define VSMC_STATIC_ASSERT_STATE_CL_VALUE_TYPE(type)                          \
    VSMC_STATIC_ASSERT((cxx11::is_same<type, cl_float>::value                 \
                || cxx11::is_same<type, cl_double>::value),                   \
            USE_StateCL_WITH_A_STATE_TYPE_OTHER_THAN_cl_float_AND_cl_double)

#define VSMC_STATIC_ASSERT_STATE_TYPE(base, derived, user)                    \
    VSMC_STATIC_ASSERT((vsmc::traits::IsBaseOfState<base, derived>::value),   \
            USE_##user##_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_##base)

#endif // VSMC_INTERNAL_ASSERT_HPP
