#ifndef VSMC_INTERNAL_ASSERT_HPP
#define VSMC_INTERNAL_ASSERT_HPP

#include <vsmc/internal/config.hpp>

#include <cassert>
#include <cstdio>
#include <stdexcept>

// Runtime assertion

#if VSMC_RUNTIME_ASSERT_AS_EXCEPTION
#define VSMC_RUNTIME_ASSERT(cond, msg)                                       \
{                                                                            \
    if (!(cond)) {                                                           \
        throw ::vsmc::RuntimeAssert(msg);                                    \
    };                                                                       \
}
#elif defined(NDEBUG) // No Debug
#define VSMC_RUNTIME_ASSERT(cond, msg)
#else // Runtime assertion
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

#if VSMC_RUNTIME_WARNING_AS_EXCEPTION
#define VSMC_RUNTIME_WARNING(cond, msg)                                      \
{                                                                            \
    if (!(cond)) {                                                           \
        throw ::vsmc::RuntimeWarning(msg);                                   \
    };                                                                       \
}
#elif defined(NDEBUG) // No Debug
#define VSMC_RUNTIME_WARNING(cond, msg)
#else // Runtime warning
#define VSMC_RUNTIME_WARNING(cond, msg)                                      \
{                                                                            \
    if (!(cond)) {                                                           \
        std::fprintf(stderr,                                                 \
                "vSMC runtime warning; File: %s; Line: %d\n%s\n",            \
                __FILE__, __LINE__, msg);                                    \
    };                                                                       \
}
#endif // VSMC_RUNTIME_WARNING_AS_EXCEPTION

// Static assertion

#if VSMC_HAS_CXX11_STATIC_ASSERT
#define VSMC_STATIC_ASSERT(cond, msg) static_assert(cond, #msg)
#else // VSMC_HAS_CXX11_STATIC_ASSERT
#define VSMC_STATIC_ASSERT(cond, msg) \
{                                                                            \
    struct VSMC_STATIC_ASSERT_FAILURE_##msg {};                              \
    ::vsmc::StaticAssert<bool(cond)>::test(                                  \
                VSMC_STATIC_ASSERT_FAILURE_##msg());                         \
}
#endif // VSMC_HAS_CXX11_STATIC_ASSERT

namespace vsmc {

class RuntimeAssert : public std::runtime_error
{
    public :

    RuntimeAssert (const std::string &msg) : std::runtime_error(msg) {}
    RuntimeAssert (const char *msg) : std::runtime_error(msg) {}
}; // class RuntimeAssert

class RuntimeWarning : public std::runtime_error
{
    public :

    RuntimeWarning (const std::string &msg) : std::runtime_error(msg) {}
    RuntimeWarning (const char *msg) : std::runtime_error(msg) {}
}; // class RuntimeWarning

template <bool> struct StaticAssert {static void test (int) {}};

template <>
struct StaticAssert<true> {static void test (...) {}};

} // namespace vsmc

#endif // VSMC_INTERNAL_ASSERT_HPP
