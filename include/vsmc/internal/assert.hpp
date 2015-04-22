//============================================================================
// vSMC/include/vsmc/internal/assert.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
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

#include <vsmc/internal/config.hpp>

#include <cassert>
#include <cstdio>
#include <stdexcept>
#include <string>

#define VSMC_STATIC_ASSERT(cond, msg) static_assert(cond, #msg)

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
            std::fprintf(stderr,                                              \
                "vSMC runtime assertion failed; File: %s; Line: %d\n%s\n",    \
                __FILE__, __LINE__, msg);                                     \
            std::fflush(stderr);                                              \
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

namespace vsmc
{

namespace internal
{

template <bool>
struct StaticAssert {
    static void test(int *) {}
};

template <>
struct StaticAssert<true> {
    static void test(...) {}
};

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
