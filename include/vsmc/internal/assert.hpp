#ifndef VSMC_INTERNAL_ASSERT_HPP
#define VSMC_INTERNAL_ASSERT_HPP

#include <cassert>
#include <cstdio>
#include <stdexcept>

#include <vsmc/internal/config.hpp>

// Runtime assertion
#if VSMC_RUNTIME_ASSERT_AS_EXCEPTION
#define VSMC_RUNTIME_ASSERT(cond, msg)                                       \
{                                                                            \
    if (!(cond)) {                                                           \
        throw vsmc::RuntimeAssert(msg);                                      \
    };                                                                       \
}
#elif defined(NDEBUG) // VSMC_RUNTIME_ASSERT_AS_EXCEPTION
#define VSMC_RUNTIME_ASSERT(cond, msg)
#else
#define VSMC_RUNTIME_ASSERT(cond, msg)                                       \
{                                                                            \
    if (!(cond)) {                                                           \
        std::fprintf(stderr,                                                 \
                "vSMC runtime assertion failed; File: %s; Line: %d\n%s\n",   \
                __FILE__, __LINE__, msg);                                    \
    };                                                                       \
    assert(cond);                                                            \
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

        USE_InitializeAdapter_WITHOUT_AN_INITIAILIZE_IMPLEMENTATION,
        USE_MoveAdapter_WITHOUT_A_MOVE_IMPLEMENTATION,
        USE_MonitorEvalAdapter_WITHOUT_A_MONITOR_EVAL_IMPLEMENTATION,
        USE_PathEvalAdapter_WITHOUT_A_PATH_EVAL_IMPLEMENTATION,

        NO_IMPLEMENTATION_OF_initialize_state_FOUND,
        NO_IMPLEMENTATION_OF_move_state_FOUND,
        NO_IMPLEMENTATION_OF_monitor_state_FOUND,
        NO_IMPLEMENTATION_OF_path_state_FOUND,
        NO_IMPLEMENTATION_OF_path_width_FOUND
    };
}; // class StaticAssert

} // namespace vsmc

#define VSMC_STATIC_ASSERT_STATE_TYPE(base, derived, user)                   \
    VSMC_STATIC_ASSERT((vsmc::traits::IsBaseOfState<base, derived>::value),  \
            USE_##user##_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_##base)

#define VSMC_STATIC_ASSERT_STATE_CL_VALUE_TYPE(type)                         \
    VSMC_STATIC_ASSERT((cxx11::is_same<type, cl_float>::value                \
             || cxx11::is_same<type, cl_double>::value),                     \
            USE_StateCL_WITH_A_STATE_TYPE_OTHER_THAN_cl_float_AND_cl_double)

#define VSMC_STATIC_ASSERT_STATE_CL_TYPE(derived, user)                      \
    VSMC_STATIC_ASSERT((vsmc::traits::IsBaseOfStateCL<derived>::value),      \
            USE_##user##_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL)

#define VSMC_STATIC_ASSERT_NO_IMPL(member)                   \
    VSMC_STATIC_ASSERT(false, NO_IMPLEMENTATION_OF_##member##_FOUND)

#define VSMC_RUNTIME_ASSERT_STATE_CL_BUILD(func)                             \
    VSMC_RUNTIME_ASSERT((build()),                                           \
            ("**StateCL::"#func"** can only be called after true "           \
             "**StateCL::build**"));

#define VSMC_RUNTIME_ASSERT_CONST_SINGLE_PARTICLE_VALID                      \
    VSMC_RUNTIME_ASSERT(particle_ptr_,                                       \
            ("A **ConstSingleParticle** object "                             \
             "is contructed with 0 **Particle** pointer"));                  \
    VSMC_RUNTIME_ASSERT((id_ >= 0 && id_ <= particle_ptr_->size()),          \
            ("A **ConstSignleParticle** object "                             \
             "is contructed with an out of range id"));

#define VSMC_RUNTIME_ASSERT_SINGLE_PARTICLE_VALID                            \
    VSMC_RUNTIME_ASSERT(particle_ptr_,                                       \
            ("A **SingleParticle** object "                                  \
             "is contructed with 0 **Particle** pointer"));                  \
    VSMC_RUNTIME_ASSERT((id_ >= 0 && id_ <= particle_ptr_->size()),          \
            ("A **SignleParticle** object "                                  \
             "is contructed with an out of range id"));

#define VSMC_RUNTIME_ASSERT_DERIVED_BASE(basename)                           \
    VSMC_RUNTIME_ASSERT((dynamic_cast<Derived *>(this)),                     \
            ("DERIVED FROM " #basename                                       \
            " WITH INCORRECT **Derived** TEMPLATE PARAMTER"));

#define VSMC_RUNTIME_ASSERT_CL_MANAGER_SETUP(func)                           \
    VSMC_RUNTIME_ASSERT((setup()),                                           \
            ("**vsmc::Manager::"#func"** can only be called after true "   \
             "**vsmc::Manager::setup**"));

#define VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status)                  \
    VSMC_RUNTIME_ASSERT((status == VSL_STATUS_OK),                           \
            ("CALLING **vsldSSEditTask** failed"));

#endif // VSMC_INTERNAL_ASSERT_HPP
