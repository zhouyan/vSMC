//============================================================================
// vSMC/include/vsmc/internal/assert.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_INTERNAL_ASSERT_HPP
#define VSMC_INTERNAL_ASSERT_HPP

#include <vsmc/internal/config.h>

#include <cassert>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <type_traits>

#define VSMC_STATIC_ASSERT(cond, msg) static_assert(cond, msg)

#if VSMC_NO_RUNTIME_ASSERT
#define VSMC_RUNTIME_ASSERT(cond, msg)
#else // VSMC_NO_RUNTIME_ASSERT
#if VSMC_RUNTIME_ASSERT_AS_EXCEPTION
#define VSMC_RUNTIME_ASSERT(cond, msg)                                        \
    {                                                                         \
        if (!(cond)) {                                                        \
            throw vsmc::RuntimeAssert(msg);                                   \
        };                                                                    \
    }
#else // VSMC_RUNTIME_ASSERT_AS_EXCEPTION
#define VSMC_RUNTIME_ASSERT(cond, msg)                                        \
    {                                                                         \
        if (!(cond)) {                                                        \
            std::fprintf(stderr, "vSMC runtime assertion failded:%s:%d:%s\n", \
                __FILE__, __LINE__, msg);                                     \
        };                                                                    \
        assert(cond);                                                         \
    }
#endif // VSMC_RUNTIME_ASSERT_AS_EXCEPTION
#endif // VSMC_NO_RUNTIME_ASSERT

#if VSMC_NO_RUNTIME_WARNING
#define VSMC_RUNTIME_WARNING(cond, msg)
#else // VSMC_NO_RUNTIME_WARNING
#if VSMC_RUNTIME_WARNING_AS_EXCEPTION
#define VSMC_RUNTIME_WARNING(cond, msg)                                       \
    {                                                                         \
        if (!(cond)) {                                                        \
            throw vsmc::RuntimeWarning(msg);                                  \
        };                                                                    \
    }
#else // VSMC_RUNTIME_WARNING_AS_EXCEPTION
#define VSMC_RUNTIME_WARNING(cond, msg)                                       \
    {                                                                         \
        if (!(cond)) {                                                        \
            std::fprintf(stderr,                                              \
                "vSMC runtime warning; File: %s; Line: %d\n%s\n", __FILE__,   \
                __LINE__, msg);                                               \
            std::fflush(stderr);                                              \
        };                                                                    \
    }
#endif // VSMC_RUNTIME_WARNING_AS_EXCEPTION
#endif // VSMC_NO_RUNTIME_WARNING

#define VSMC_STATIC_ASSERT_TEMPLATE_TYPE_1(Temp, T, T1)                       \
    VSMC_STATIC_ASSERT((std::is_same<T, T1>::value),                          \
        "**" #Temp "** USED WITH " #T " OTHER THAN " #T1)

#define VSMC_STATIC_ASSERT_TEMPLATE_TYPE_2(Temp, T, T1, T2)                   \
    VSMC_STATIC_ASSERT(                                                       \
        (std::is_same<T, T1>::value || std::is_same<T, T2>::value),           \
        "**" #Temp "** USED WITH " #T " OTHER THAN " #T1 " OR " #T2)

#define VSMC_STATIC_ASSERT_TEMPLATE_TYPE_3(Temp, T, T1, T2, T3)               \
    VSMC_STATIC_ASSERT(                                                       \
        (std::is_same<T, T1>::value || std::is_same<T, T2>::value ||          \
            std::is_same<T, T3>::value),                                      \
        "**" #Temp "** USED WITH " #T " OTHER THAN " #T1 " OR " #T2           \
        " OR " #T3)

#define VSMC_STATIC_ASSERT_TEMPLATE_TYPE_4(Temp, T, T1, T2, T3, T4)           \
    VSMC_STATIC_ASSERT(                                                       \
        (std::is_same<T, T1>::value || std::is_same<T, T2>::value ||          \
            std::is_same<T, T3>::value || std::is_same<T, T4>::value),        \
        "**" #Temp "** USED WITH " #T " OTHER THAN " #T1 " OR " #T2           \
        " OR " #T3 " OR " #T4)

namespace vsmc
{

namespace internal
{

template <bool>
class StaticAssert
{
    public:
    static void test(int *) {}
}; // class StaticAssert

template <>
class StaticAssert<true>
{
    public:
    static void test(...) {}
}; // class StaticAssert

} // namespace vsmc::internal

class RuntimeAssert : public std::runtime_error
{
    public:
    explicit RuntimeAssert(const char *msg) : std::runtime_error(msg) {}

    explicit RuntimeAssert(const std::string &msg) : std::runtime_error(msg) {}
}; // class RuntimeAssert

class RuntimeWarning : public std::runtime_error
{
    public:
    explicit RuntimeWarning(const char *msg) : std::runtime_error(msg) {}

    explicit RuntimeWarning(const std::string &msg) : std::runtime_error(msg)
    {
    }
}; // class RuntimeWarning

} // namespace vsmc

#endif // VSMC_INTERNAL_ASSERT_HPP
