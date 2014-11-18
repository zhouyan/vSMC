//============================================================================
// vSMC/include/vsmc/utility/cstring.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
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

/// \addtogroup CString
///
/// This module implement the `memcpy`, etc., functions in the `vsmc`
/// namespace. The implementaions are optimzied with SIMD instructions. Three
/// groups of functions are provided.
///
/// - `memcpy_std` etc., they simply call `std::memcpy` etc.
/// - `memcpy_sse2` etc., they are avialable if at least SSE2 is supported and
/// are optimized with SSE2 instructions
/// - `memcpy_avx` etc., they are avialable if at least AVX is supported and
/// are optimized with AVX instructions
///
/// There are also generic `vsmc::memcpy` etc. They dispatch the call based on
/// the following rules.
///
/// - If AVX is available, then call `memcpy_avx` etc.
/// - Else if SSE2 is available, then call `memcpy_sse2` etc.
/// - Else call `memcpy_std`.
///
/// This dispatch can be done at compile time if the configuration macro
/// `VSMC_CSTRING_RUNTIME_DISPATCH` is zero. If the macro is non-zero, then it
/// will be done at runtime using `CPUID` information.
///
/// Before using any of these vSMC provided functions. A few factors shall be
/// considered.
///
/// - For large buffers, vSMC functions use non-temporal store instructions. In
/// this case, the performance is likely to be better than the system library
/// or compiler builtins. On Haswell and Nehalem, about 30% performance gain
/// were observed, though 70% were observed in some situations. A more
/// important benefits is that cache pollution may be avoided in this case. The
/// threshold is set to be half the size of the LLC (last level cache). Note
/// that, only up to level 3 cache is considered. Some newer Intel CPUs have
/// level 4 cache, which is shared with the integrated GPU. This threshold can
/// be changed at compile time by define `VSMC_CSTRING_NON_TEMPORAL_THRESHOLD`
/// or at runtime via CStringNonTemporalThreshold singleton.
/// - For moderate size buffers (from 1KB upto the non-temporal threshold), the
/// performance is most likely to be only comparable to the system. At worst,
/// 20% perforamnce degrade was observed, though in most cases, it is almost as
/// fast as or slightly faster than system library. To change the threshold,
/// ~~~{.cpp}
/// vsmc::CStringNonTemporalThreshold::instance().set(new_threshold);
/// ~~~
/// To change the maximum level of cache to be considered and reset the
/// threshold accordingly.
/// ~~~{.cpp}
/// vsmc::CStringNonTemporalThreshold::instance().max_level(new_level);
/// ~~~
/// - The performance is only tested on limited models of CPUs. As a single
/// person I don't have much resources. In particle, AMD CPUs were not tested
/// at all.
/// - At the moment, `vsmc::memcpy` is more optimized than `vsmc::memmove`.
/// System `memmove` is likely to outpeform `vsmc::memmove` considerably except
/// for large buffers.
///
/// In any case, most systems's standard C library is likely to be optimized
/// enough to suffice most usage situations. And even when there is a
/// noticeable perforamnce difference, unless all the program do is copy and
/// moving memories, the difference is not likely to be big enough to make a
/// difference. And taking caching into consideration, those difference seem in
/// memory dedicated benchmarks might well not exist at all in real programs.
///
/// In summary, do some benchmark of real programs before deciding if using
/// these functions are beneficial.

#ifndef VSMC_UTILITY_CSTRING_HPP
#define VSMC_UTILITY_CSTRING_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/cpuid.hpp>

#if VSMC_HAS_SSE2
#include <emmintrin.h>
#endif

#if VSMC_HAS_AVX
#include <immintrin.h>
#endif

/// \brief Shall functions in this module do runtime dispatch
/// \ingroup Config
#ifndef VSMC_CSTRING_RUNTIME_DISPATCH
#define VSMC_CSTRING_RUNTIME_DISPATCH 0
#endif

/// \brief Threshold above which non-temporal copy shall be used (0 for auto)
/// \ingroup Config
#ifndef VSMC_CSTRING_NON_TEMPORAL_THRESHOLD
#define VSMC_CSTRING_NON_TEMPORAL_THRESHOLD 0
#endif

#define VSMC_RUNTIME_ASSERT_UTILITY_CSTRING_SWITCH(func) \
    VSMC_RUNTIME_ASSERT(false, ("**vsmc::internal::"#func" UNDEFINED FLAG"))

#define VSMC_RUNTIME_ASSERT_UTILITY_CSTRING_MEMCPY(dst, src, n) \
    VSMC_RUNTIME_ASSERT((                                                    \
                static_cast<const char *>(dst) -                             \
                static_cast<const char *>(src) <=                            \
                static_cast<std::ptrdiff_t>(n) &&                            \
                static_cast<const char *>(src) -                             \
                static_cast<const char *>(dst) <=                            \
                static_cast<std::ptrdiff_t>(n)),                             \
            ("**vsmc::memcpy** OVERLAPPING BUFFERS"))

#define VSMC_DEFINE_UTILITY_CSTRING_SET(ISA, da, nt, c, m,\
        cast, set1, store) \
template <>                                                                  \
VSMC_STRONG_INLINE void set_n<ISA, 1, da, nt> (                              \
        void *dst, int ch, std::size_t n)                                    \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    m m0 = cast(set1(static_cast<char>(static_cast<unsigned char>(ch))));    \
    if (n >= traits::SIMDTrait<ISA>::alignment) {                            \
        n -= traits::SIMDTrait<ISA>::alignment;                              \
        store(dstc, m0);                                                     \
        dstc += traits::SIMDTrait<ISA>::alignment / sizeof(c);               \
    }                                                                        \
    set_0<ISA>(dstc, ch, n);                                                 \
}                                                                            \
                                                                             \
template <>                                                                  \
VSMC_STRONG_INLINE void set_n<ISA, 2, da, nt> (                              \
        void *dst, int ch, std::size_t n)                                    \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    m m0 = cast(set1(static_cast<char>(static_cast<unsigned char>(ch))));    \
    if (n >= traits::SIMDTrait<ISA>::alignment * 2) {                        \
        n -= traits::SIMDTrait<ISA>::alignment * 2;                          \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 0 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c), m0); \
        dstc += traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c);           \
    }                                                                        \
    set_n<ISA, 1, da, nt>(dstc, ch, n);                                      \
}                                                                            \
                                                                             \
template <>                                                                  \
VSMC_STRONG_INLINE void set_n<ISA, 4, da, nt> (                              \
        void *dst, int ch, std::size_t n)                                    \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    m m0 = cast(set1(static_cast<char>(static_cast<unsigned char>(ch))));    \
    if (n >= traits::SIMDTrait<ISA>::alignment * 4) {                        \
        n -= traits::SIMDTrait<ISA>::alignment * 4;                          \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 0 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c), m0); \
        dstc += traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c);           \
    }                                                                        \
    set_n<ISA, 2, da, nt>(dstc, ch, n);                                      \
}                                                                            \
                                                                             \
template <>                                                                  \
VSMC_STRONG_INLINE void set_n<ISA, 8, da, nt> (                              \
        void *dst, int ch, std::size_t n)                                    \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    m m0 = cast(set1(static_cast<char>(static_cast<unsigned char>(ch))));    \
    if (n >= traits::SIMDTrait<ISA>::alignment * 8) {                        \
        n -= traits::SIMDTrait<ISA>::alignment * 8;                          \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 0 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 5 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 6 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 7 / sizeof(c), m0); \
        dstc += traits::SIMDTrait<ISA>::alignment * 8 / sizeof(c);           \
    }                                                                        \
    set_n<ISA, 4, da, nt>(dstc, ch, n);                                      \
}                                                                            \
                                                                             \
template <>                                                                  \
VSMC_STRONG_INLINE void set_l<ISA, 4, da, nt> (                              \
        void *dst, int ch, std::size_t n)                                    \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    m m0 = cast(set1(static_cast<char>(static_cast<unsigned char>(ch))));    \
    while (n >= traits::SIMDTrait<ISA>::alignment * 4) {                     \
        n -= traits::SIMDTrait<ISA>::alignment * 4;                          \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 0 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c), m0); \
        dstc += traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c);           \
    }                                                                        \
    set_n<ISA, 2, da, nt>(dstc, ch, n);                                      \
}                                                                            \
                                                                             \
template <>                                                                  \
VSMC_STRONG_INLINE void set_l<ISA, 8, da, nt> (                              \
        void *dst, int ch, std::size_t n)                                    \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    m m0 = cast(set1(static_cast<char>(static_cast<unsigned char>(ch))));    \
    while (n >= traits::SIMDTrait<ISA>::alignment * 8) {                     \
        n -= traits::SIMDTrait<ISA>::alignment * 8;                          \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 0 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 5 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 6 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 7 / sizeof(c), m0); \
        dstc += traits::SIMDTrait<ISA>::alignment * 8 / sizeof(c);           \
    }                                                                        \
    set_n<ISA, 4, da, nt>(dstc, ch, n);                                      \
}

#define VSMC_DEFINE_UTILITY_CSTRING_CPY(ISA, sa, da, nt, c, m, load, store) \
template <>                                                                  \
VSMC_STRONG_INLINE void cpy_front_n<ISA, 1, sa, da, nt> (                    \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    const c *srcc = static_cast<const c *>(src);                             \
    if (n >= traits::SIMDTrait<ISA>::alignment) {                            \
        n -= traits::SIMDTrait<ISA>::alignment;                              \
        m m0 = load(srcc);                                                   \
        store(dstc, m0);                                                     \
        dstc += traits::SIMDTrait<ISA>::alignment / sizeof(c);               \
        srcc += traits::SIMDTrait<ISA>::alignment / sizeof(c);               \
    }                                                                        \
    cpy_front_0<ISA>(dstc, srcc, n);                                         \
}                                                                            \
                                                                             \
template <>                                                                  \
VSMC_STRONG_INLINE void cpy_front_n<ISA, 2, sa, da, nt> (                    \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    const c *srcc = static_cast<const c *>(src);                             \
    if (n >= traits::SIMDTrait<ISA>::alignment * 2) {                        \
        n -= traits::SIMDTrait<ISA>::alignment * 2;                          \
        m m0 = load(srcc + traits::SIMDTrait<ISA>::alignment * 0 / sizeof(c));\
        m m1 = load(srcc + traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c));\
        store(dstc + traits::SIMDTrait<ISA>::alignment * 0 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c), m1); \
        dstc += traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c);           \
        srcc += traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c);           \
    }                                                                        \
    cpy_front_n<ISA, 1, sa, da, nt>(dstc, srcc, n);                          \
}                                                                            \
                                                                             \
                                                                             \
template <>                                                                  \
VSMC_STRONG_INLINE void cpy_front_n<ISA, 4, sa, da, nt> (                    \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    const c *srcc = static_cast<const c *>(src);                             \
    if (n >= traits::SIMDTrait<ISA>::alignment * 4) {                        \
        n -= traits::SIMDTrait<ISA>::alignment * 4;                          \
        m m0 = load(srcc + traits::SIMDTrait<ISA>::alignment * 0 / sizeof(c));\
        m m1 = load(srcc + traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c));\
        m m2 = load(srcc + traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c));\
        m m3 = load(srcc + traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c));\
        store(dstc + traits::SIMDTrait<ISA>::alignment * 0 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c), m1); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c), m2); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c), m3); \
        dstc += traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c);           \
        srcc += traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c);           \
    }                                                                        \
    cpy_front_n<ISA, 2, sa, da, nt>(dstc, srcc, n);                          \
}                                                                            \
                                                                             \
template <>                                                                  \
VSMC_STRONG_INLINE void cpy_front_n<ISA, 8, sa, da, nt> (                    \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    const c *srcc = static_cast<const c *>(src);                             \
    if (n >= traits::SIMDTrait<ISA>::alignment * 8) {                        \
        n -= traits::SIMDTrait<ISA>::alignment * 8;                          \
        m m0 = load(srcc + traits::SIMDTrait<ISA>::alignment * 0 / sizeof(c));\
        m m1 = load(srcc + traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c));\
        m m2 = load(srcc + traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c));\
        m m3 = load(srcc + traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c));\
        m m4 = load(srcc + traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c));\
        m m5 = load(srcc + traits::SIMDTrait<ISA>::alignment * 5 / sizeof(c));\
        m m6 = load(srcc + traits::SIMDTrait<ISA>::alignment * 6 / sizeof(c));\
        m m7 = load(srcc + traits::SIMDTrait<ISA>::alignment * 7 / sizeof(c));\
        store(dstc + traits::SIMDTrait<ISA>::alignment * 0 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c), m1); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c), m2); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c), m3); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c), m4); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 5 / sizeof(c), m5); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 6 / sizeof(c), m6); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 7 / sizeof(c), m7); \
        dstc += traits::SIMDTrait<ISA>::alignment * 8 / sizeof(c);           \
        srcc += traits::SIMDTrait<ISA>::alignment * 8 / sizeof(c);           \
    }                                                                        \
    cpy_front_n<ISA, 4, sa, da, nt>(dstc, srcc, n);                          \
}                                                                            \
                                                                             \
template <>                                                                  \
VSMC_STRONG_INLINE void cpy_front_l<ISA, 4, sa, da, nt> (                    \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    const c *srcc = static_cast<const c *>(src);                             \
    while (n >= traits::SIMDTrait<ISA>::alignment * 4) {                     \
        n -= traits::SIMDTrait<ISA>::alignment * 4;                          \
        m m0 = load(srcc + traits::SIMDTrait<ISA>::alignment * 0 / sizeof(c));\
        m m1 = load(srcc + traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c));\
        m m2 = load(srcc + traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c));\
        m m3 = load(srcc + traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c));\
        store(dstc + traits::SIMDTrait<ISA>::alignment * 0 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c), m1); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c), m2); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c), m3); \
        dstc += traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c);           \
        srcc += traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c);           \
    }                                                                        \
    cpy_front_n<ISA, 2, sa, da, nt>(dstc, srcc, n);                          \
}                                                                            \
                                                                             \
template <>                                                                  \
VSMC_STRONG_INLINE void cpy_front_l<ISA, 8, sa, da, nt> (                    \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    const c *srcc = static_cast<const c *>(src);                             \
    while (n >= traits::SIMDTrait<ISA>::alignment * 8) {                     \
        n -= traits::SIMDTrait<ISA>::alignment * 8;                          \
        m m0 = load(srcc + traits::SIMDTrait<ISA>::alignment * 0 / sizeof(c));\
        m m1 = load(srcc + traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c));\
        m m2 = load(srcc + traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c));\
        m m3 = load(srcc + traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c));\
        m m4 = load(srcc + traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c));\
        m m5 = load(srcc + traits::SIMDTrait<ISA>::alignment * 5 / sizeof(c));\
        m m6 = load(srcc + traits::SIMDTrait<ISA>::alignment * 6 / sizeof(c));\
        m m7 = load(srcc + traits::SIMDTrait<ISA>::alignment * 7 / sizeof(c));\
        store(dstc + traits::SIMDTrait<ISA>::alignment * 0 / sizeof(c), m0); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c), m1); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c), m2); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c), m3); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c), m4); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 5 / sizeof(c), m5); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 6 / sizeof(c), m6); \
        store(dstc + traits::SIMDTrait<ISA>::alignment * 7 / sizeof(c), m7); \
        dstc += traits::SIMDTrait<ISA>::alignment * 8 / sizeof(c);           \
        srcc += traits::SIMDTrait<ISA>::alignment * 8 / sizeof(c);           \
    }                                                                        \
    cpy_front_n<ISA, 4, sa, da, nt>(dstc, srcc, n);                          \
}                                                                            \
                                                                             \
template <>                                                                  \
VSMC_STRONG_INLINE void cpy_back_n<ISA, 1, sa, da, nt> (                     \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    const c *srcc = static_cast<const c *>(src);                             \
    if (n >= traits::SIMDTrait<ISA>::alignment) {                            \
        n -= traits::SIMDTrait<ISA>::alignment;                              \
        m m1 = load(srcc - traits::SIMDTrait<ISA>::alignment / sizeof(c));   \
        store(dstc - traits::SIMDTrait<ISA>::alignment / sizeof(c), m1);     \
        dstc -= traits::SIMDTrait<ISA>::alignment / sizeof(c);               \
        srcc -= traits::SIMDTrait<ISA>::alignment / sizeof(c);               \
    }                                                                        \
    cpy_back_0<ISA>(dstc, srcc, n);                                          \
}                                                                            \
                                                                             \
template <>                                                                  \
VSMC_STRONG_INLINE void cpy_back_n<ISA, 2, sa, da, nt> (                     \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    const c *srcc = static_cast<const c *>(src);                             \
    if (n >= traits::SIMDTrait<ISA>::alignment * 2) {                        \
        n -= traits::SIMDTrait<ISA>::alignment * 2;                          \
        m m1 = load(srcc - traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c));\
        m m2 = load(srcc - traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c));\
        store(dstc - traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c), m1); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c), m2); \
        dstc -= traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c);           \
        srcc -= traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c);           \
    }                                                                        \
    cpy_back_n<ISA, 1, sa, da, nt>(dstc, srcc, n);                           \
}                                                                            \
                                                                             \
template <>                                                                  \
VSMC_STRONG_INLINE void cpy_back_n<ISA, 4, sa, da, nt> (                     \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    const c *srcc = static_cast<const c *>(src);                             \
    if (n >= traits::SIMDTrait<ISA>::alignment * 4) {                        \
        n -= traits::SIMDTrait<ISA>::alignment * 4;                          \
        m m1 = load(srcc - traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c));\
        m m2 = load(srcc - traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c));\
        m m3 = load(srcc - traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c));\
        m m4 = load(srcc - traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c));\
        store(dstc - traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c), m1); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c), m2); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c), m3); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c), m4); \
        dstc -= traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c);           \
        srcc -= traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c);           \
    }                                                                        \
    cpy_back_n<ISA, 2, sa, da, nt>(dstc, srcc, n);                           \
}                                                                            \
                                                                             \
template <>                                                                  \
VSMC_STRONG_INLINE void cpy_back_n<ISA, 8, sa, da, nt> (                     \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    const c *srcc = static_cast<const c *>(src);                             \
    if (n >= traits::SIMDTrait<ISA>::alignment * 8) {                        \
        n -= traits::SIMDTrait<ISA>::alignment * 8;                          \
        m m1 = load(srcc - traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c));\
        m m2 = load(srcc - traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c));\
        m m3 = load(srcc - traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c));\
        m m4 = load(srcc - traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c));\
        m m5 = load(srcc - traits::SIMDTrait<ISA>::alignment * 5 / sizeof(c));\
        m m6 = load(srcc - traits::SIMDTrait<ISA>::alignment * 6 / sizeof(c));\
        m m7 = load(srcc - traits::SIMDTrait<ISA>::alignment * 7 / sizeof(c));\
        m m8 = load(srcc - traits::SIMDTrait<ISA>::alignment * 8 / sizeof(c));\
        store(dstc - traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c), m1); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c), m2); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c), m3); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c), m4); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 5 / sizeof(c), m5); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 6 / sizeof(c), m6); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 7 / sizeof(c), m7); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 8 / sizeof(c), m8); \
        dstc -= traits::SIMDTrait<ISA>::alignment * 8 / sizeof(c);           \
        srcc -= traits::SIMDTrait<ISA>::alignment * 8 / sizeof(c);           \
    }                                                                        \
    cpy_back_n<ISA, 4, sa, da, nt>(dstc, srcc, n);                           \
}                                                                            \
                                                                             \
template <>                                                                  \
VSMC_STRONG_INLINE void cpy_back_l<ISA, 4, sa, da, nt> (                     \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    const c *srcc = static_cast<const c *>(src);                             \
    while (n >= traits::SIMDTrait<ISA>::alignment * 4) {                     \
        n -= traits::SIMDTrait<ISA>::alignment * 4;                          \
        m m1 = load(srcc - traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c));\
        m m2 = load(srcc - traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c));\
        m m3 = load(srcc - traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c));\
        m m4 = load(srcc - traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c));\
        store(dstc - traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c), m1); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c), m2); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c), m3); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c), m4); \
        dstc -= traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c);           \
        srcc -= traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c);           \
    }                                                                        \
    cpy_back_n<ISA, 2, sa, da, nt>(dstc, srcc, n);                           \
}                                                                            \
                                                                             \
template <>                                                                  \
VSMC_STRONG_INLINE void cpy_back_l<ISA, 8, sa, da, nt> (                     \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    const c *srcc = static_cast<const c *>(src);                             \
    while (n >= traits::SIMDTrait<ISA>::alignment * 8) {                     \
        n -= traits::SIMDTrait<ISA>::alignment * 8;                          \
        m m1 = load(srcc - traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c));\
        m m2 = load(srcc - traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c));\
        m m3 = load(srcc - traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c));\
        m m4 = load(srcc - traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c));\
        m m5 = load(srcc - traits::SIMDTrait<ISA>::alignment * 5 / sizeof(c));\
        m m6 = load(srcc - traits::SIMDTrait<ISA>::alignment * 6 / sizeof(c));\
        m m7 = load(srcc - traits::SIMDTrait<ISA>::alignment * 7 / sizeof(c));\
        m m8 = load(srcc - traits::SIMDTrait<ISA>::alignment * 8 / sizeof(c));\
        store(dstc - traits::SIMDTrait<ISA>::alignment * 1 / sizeof(c), m1); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 2 / sizeof(c), m2); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 3 / sizeof(c), m3); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 4 / sizeof(c), m4); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 5 / sizeof(c), m5); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 6 / sizeof(c), m6); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 7 / sizeof(c), m7); \
        store(dstc - traits::SIMDTrait<ISA>::alignment * 8 / sizeof(c), m8); \
        dstc -= traits::SIMDTrait<ISA>::alignment * 8 / sizeof(c);           \
        srcc -= traits::SIMDTrait<ISA>::alignment * 8 / sizeof(c);           \
    }                                                                        \
    cpy_back_n<ISA, 4, sa, da, nt>(dstc, srcc, n);                           \
}

namespace vsmc {

namespace internal {

template <SIMD ISA>
VSMC_STRONG_INLINE unsigned cstring_is_aligned (const void *ptr)
{
    return reinterpret_cast<uintptr_t>(ptr) %
        traits::SIMDTrait<ISA>::alignment == 0 ? 2 : 0;
}

template <SIMD>
VSMC_STRONG_INLINE void set_0 (void *dst, int ch, std::size_t n)
{
    if (n != 0)
        std::memset(dst, ch, n);
}

template <SIMD>
VSMC_STRONG_INLINE void cpy_front_0 (void *dst, const void *src, std::size_t n)
{
    if (n != 0)
        std::memmove(dst, src, n);
}

template <SIMD>
VSMC_STRONG_INLINE void cpy_back_0 (void *dst, const void *src, std::size_t n)
{
    if (n != 0) {
        dst = static_cast<void *>(static_cast<char *>(dst) - n);
        src = static_cast<const void *>(static_cast<const char *>(src) - n);
        std::memmove(dst, src, n);
    }
}

template <SIMD, std::size_t, bool, bool>
VSMC_STRONG_INLINE void set_n (void *, int, std::size_t);
template <SIMD, std::size_t, bool, bool>
VSMC_STRONG_INLINE void set_l (void *, int, std::size_t);

template <SIMD, std::size_t, bool, bool, bool>
VSMC_STRONG_INLINE void cpy_front_n (void *, const void *, std::size_t);
template <SIMD, std::size_t, bool, bool, bool>
VSMC_STRONG_INLINE void cpy_front_l (void *, const void *, std::size_t);

template <SIMD, std::size_t, bool, bool, bool>
VSMC_STRONG_INLINE void cpy_back_n (void *, const void *, std::size_t);
template <SIMD, std::size_t, bool, bool, bool>
VSMC_STRONG_INLINE void cpy_back_l (void *, const void *, std::size_t);

#if VSMC_HAS_SSE2

VSMC_DEFINE_UTILITY_CSTRING_SET(SSE2, false, false,
        double, __m128d, _mm_castsi128_pd, _mm_set1_epi8,
        _mm_storeu_pd)
VSMC_DEFINE_UTILITY_CSTRING_SET(SSE2, true, false,
        double, __m128d, _mm_castsi128_pd, _mm_set1_epi8,
        _mm_store_pd)
VSMC_DEFINE_UTILITY_CSTRING_SET(SSE2, true, true,
        double, __m128d, _mm_castsi128_pd, _mm_set1_epi8,
        _mm_stream_pd)

VSMC_DEFINE_UTILITY_CSTRING_CPY(SSE2, false, false, false,
        double, __m128d, _mm_loadu_pd, _mm_storeu_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY(SSE2, false, true, false,
        double, __m128d, _mm_loadu_pd, _mm_store_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY(SSE2, false, true, true,
        double, __m128d, _mm_loadu_pd, _mm_stream_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY(SSE2, true, false, false,
        double, __m128d, _mm_load_pd, _mm_storeu_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY(SSE2, true, true, false,
        double, __m128d, _mm_load_pd, _mm_store_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY(SSE2, true, true, true,
        double, __m128d, _mm_load_pd, _mm_stream_pd)

#endif // VSMC_HAS_SSE2

#if VSMC_HAS_AVX

VSMC_DEFINE_UTILITY_CSTRING_SET(AVX, false, false,
        double, __m256d, _mm256_castsi256_pd, _mm256_set1_epi8,
        _mm256_storeu_pd)
VSMC_DEFINE_UTILITY_CSTRING_SET(AVX, true, false,
        double, __m256d, _mm256_castsi256_pd, _mm256_set1_epi8,
        _mm256_store_pd)
VSMC_DEFINE_UTILITY_CSTRING_SET(AVX, true, true,
        double, __m256d, _mm256_castsi256_pd, _mm256_set1_epi8,
        _mm256_stream_pd)

VSMC_DEFINE_UTILITY_CSTRING_CPY(AVX, false, false, false,
        double, __m256d, _mm256_loadu_pd, _mm256_storeu_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY(AVX, false, true, false,
        double, __m256d, _mm256_loadu_pd, _mm256_store_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY(AVX, false, true, true,
        double, __m256d, _mm256_loadu_pd, _mm256_stream_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY(AVX, true, false, false,
        double, __m256d, _mm256_load_pd, _mm256_storeu_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY(AVX, true, true, false,
        double, __m256d, _mm256_load_pd, _mm256_store_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY(AVX, true, true, true,
        double, __m256d, _mm256_load_pd, _mm256_stream_pd)

#endif // VSMC_HAS_AVX

template <SIMD ISA, std::size_t N>
VSMC_STRONG_INLINE void set_n_switch (
        void *dst, int ch, std::size_t n, unsigned flag)
{
    switch (flag) {
        case 0: set_n<ISA, N, false, false>(dst, ch, n); break;
        case 2: set_n<ISA, N, true,  false>(dst, ch, n); break;
        // case 3: set_n<ISA, N, true,  true >(dst, ch, n); break;
        default :
            VSMC_RUNTIME_ASSERT_UTILITY_CSTRING_SWITCH(set_n_switch);
    }
}

template <SIMD ISA, std::size_t N>
VSMC_STRONG_INLINE void set_l_switch (
        void *dst, int ch, std::size_t n, unsigned flag)
{
    switch (flag) {
        case 0: set_l<ISA, N, false, false>(dst, ch, n); break;
        case 2: set_l<ISA, N, true,  false>(dst, ch, n); break;
        case 3: set_l<ISA, N, true,  true >(dst, ch, n); break;
        default :
            VSMC_RUNTIME_ASSERT_UTILITY_CSTRING_SWITCH(set_l_switch);
    }
}

template <SIMD ISA, std::size_t N>
VSMC_STRONG_INLINE void cpy_front_n_switch (
        void *dst, const void *src, std::size_t n, unsigned flag)
{
    switch (flag) {
        case 6 : cpy_front_n<ISA, N, true,  true,  false>(dst, src, n); break;
        case 4 : cpy_front_n<ISA, N, true,  false, false>(dst, src, n); break;
        case 2 : cpy_front_n<ISA, N, false, true,  false>(dst, src, n); break;
        case 0 : cpy_front_n<ISA, N, false, false, false>(dst, src, n); break;
        case 7 : cpy_front_n<ISA, N, true,  true,  true >(dst, src, n); break;
        case 3 : cpy_front_n<ISA, N, false, true,  true >(dst, src, n); break;
        default :
            VSMC_RUNTIME_ASSERT_UTILITY_CSTRING_SWITCH(cpy_front_n_switch);
    }
}

template <SIMD ISA, std::size_t N>
VSMC_STRONG_INLINE void cpy_front_l_switch (
        void *dst, const void *src, std::size_t n, unsigned flag)
{
    switch (flag) {
        case 0 : cpy_front_l<ISA, N, false, false, false>(dst, src, n); break;
        case 2 : cpy_front_l<ISA, N, false, true,  false>(dst, src, n); break;
        case 3 : cpy_front_l<ISA, N, false, true,  true >(dst, src, n); break;
        case 4 : cpy_front_l<ISA, N, true,  false, false>(dst, src, n); break;
        case 6 : cpy_front_l<ISA, N, true,  true,  false>(dst, src, n); break;
        case 7 : cpy_front_l<ISA, N, true,  true,  true >(dst, src, n); break;
        default :
            VSMC_RUNTIME_ASSERT_UTILITY_CSTRING_SWITCH(cpy_front_l_switch);
    }
}

template <SIMD ISA, std::size_t N>
VSMC_STRONG_INLINE void cpy_back_n_switch (
        void *dst, const void *src, std::size_t n, unsigned flag)
{
    switch (flag) {
        case 6 : cpy_back_n<ISA, N, true,  true,  false>(dst, src, n); break;
        case 4 : cpy_back_n<ISA, N, true,  false, false>(dst, src, n); break;
        case 2 : cpy_back_n<ISA, N, false, true,  false>(dst, src, n); break;
        case 0 : cpy_back_n<ISA, N, false, false, false>(dst, src, n); break;
        case 7 : cpy_back_n<ISA, N, true,  true,  true >(dst, src, n); break;
        case 3 : cpy_back_n<ISA, N, false, true,  true >(dst, src, n); break;
        default :
            VSMC_RUNTIME_ASSERT_UTILITY_CSTRING_SWITCH(cpy_back_n_switch);
    }
}

template <SIMD ISA, std::size_t N>
VSMC_STRONG_INLINE void cpy_back_l_switch (
        void *dst, const void *src, std::size_t n, unsigned flag)
{
    switch (flag) {
        case 6 : cpy_back_l<ISA, N, true,  true,  false>(dst, src, n); break;
        case 4 : cpy_back_l<ISA, N, true,  false, false>(dst, src, n); break;
        case 2 : cpy_back_l<ISA, N, false, true,  false>(dst, src, n); break;
        case 0 : cpy_back_l<ISA, N, false, false, false>(dst, src, n); break;
        case 7 : cpy_back_l<ISA, N, true,  true,  true >(dst, src, n); break;
        case 3 : cpy_back_l<ISA, N, false, true,  true >(dst, src, n); break;
        default :
            VSMC_RUNTIME_ASSERT_UTILITY_CSTRING_SWITCH(cpy_back_l_switch);
    }
}

} // namespace internal

/// \brief The threshold of buffer size above which `memcpy` use non-temporal
/// instructions.
/// \ingroup CString
class CStringNonTemporalThreshold
{
    public :

    /// \brief Singleton instance
    ///
    /// \note If any `memcpy` functions in this module is to be called from
    /// multiple thread, then this member function need to be called at least
    /// once before entering threads.
    static CStringNonTemporalThreshold &instance ()
    {
        static CStringNonTemporalThreshold ntt;

        return ntt;
    }

    /// \brief The maximum level of cache considered in `set()`.
    std::size_t max_level () const {return max_level_;}

    /// \brief Set new maximum level of cache considered in `set()`.
    ///
    /// \note
    /// This will automatically call `set()` to set the new threshold.
    void max_level (std::size_t level) {max_level_ = level; set();}

    /// \brief Set the threshold to default
    ///
    /// \details
    /// By default, we set a pretty high threshold (the LLC size).
    void set ()
    {
        unsigned cache_index = 0;
        unsigned cache_index_max = CPUID::cache_param_num();
        unsigned cache_level = 0;
        for (unsigned index = 0; index != cache_index_max; ++index) {
            unsigned level = CPUID::cache_param(index).level();
            if (level <= max_level_ && level > cache_level) {
                cache_index = index;
                cache_level = level;
            }
        }
        threshold_ = CPUID::cache_param(cache_index).size() / 2;
        if (threshold_ == 0)
            threshold_ = 1U << 18;
    }

    /// \brief Set the threshold to a specific size
    void set (std::size_t threshold) {threshold_ = threshold;}

    /// \brief Get the threshold
    std::size_t get () const {return threshold_;}

    /// \brief Give number of bytes, return flag indicate if it is over the
    /// threshold
    unsigned over (std::size_t n) const {return n > threshold_ ? 1 : 0;}

    private :

    std::size_t threshold_;
    std::size_t max_level_;

    CStringNonTemporalThreshold () :
        threshold_(VSMC_CSTRING_NON_TEMPORAL_THRESHOLD), max_level_(3)
    {if (threshold_ == 0) set();}

    CStringNonTemporalThreshold (const CStringNonTemporalThreshold &);

    CStringNonTemporalThreshold &operator= (
            const CStringNonTemporalThreshold &);
}; // class CStringNonTemporalThreshold

namespace internal {

template <SIMD ISA>
VSMC_STRONG_INLINE void *memset_simd (
        void *dst, int ch, std::size_t n)
{
    if (n < traits::SIMDTrait<ISA>::alignment) {
        set_0<ISA>(dst, ch, n);
        return dst;
    }

    unsigned flag = cstring_is_aligned<ISA>(dst);
    if (n <= traits::SIMDTrait<ISA>::alignment *
            traits::SIMDTrait<ISA>::grainsize) {
        set_n_switch<ISA, traits::SIMDTrait<ISA>::grainsize>(dst, ch, n, flag);
        return dst;
    }

    char *dstc = static_cast<char *>(dst);
    std::size_t offset = reinterpret_cast<uintptr_t>(dstc) % 64;
    if (offset != 0) {
        offset = 64 - offset;
        set_n_switch<ISA, 32 / traits::SIMDTrait<ISA>::alignment>(
                dstc, ch, offset, flag);
        n -= offset;
        dstc += offset;
    }
    flag = CStringNonTemporalThreshold::instance().over(n) | 2;
    set_l_switch<ISA, traits::SIMDTrait<ISA>::grainsize>(dstc, ch, n, flag);

    return dst;
}

template <SIMD ISA>
VSMC_STRONG_INLINE void *memcpy_simd (
        void *dst, const void *src, std::size_t n)
{
    if (dst == src)
        return dst;

    if (n < traits::SIMDTrait<ISA>::alignment) {
        cpy_front_0<ISA>(dst, src, n);
        return dst;
    }

    unsigned flag = cstring_is_aligned<ISA>(dst);
    flag |= cstring_is_aligned<ISA>(src) << 1;
    if (n <= traits::SIMDTrait<ISA>::alignment *
            traits::SIMDTrait<ISA>::grainsize) {
        cpy_front_n_switch<ISA, traits::SIMDTrait<ISA>::grainsize>(
                dst, src, n, flag);
        return dst;
    }

    char *dstc = static_cast<char *>(dst);
    const char *srcc = static_cast<const char *>(src);
    std::size_t offset = reinterpret_cast<uintptr_t>(dstc) % 64;
    if (offset != 0) {
        offset = 64 - offset;
        cpy_front_n_switch<ISA, 32 / traits::SIMDTrait<ISA>::alignment>(
                dstc, srcc, offset, flag);
        n -= offset;
        dstc += offset;
        srcc += offset;
    }
    flag = CStringNonTemporalThreshold::instance().over(n) | 2;
    flag |= cstring_is_aligned<ISA>(srcc) << 1;
    cpy_front_l_switch<ISA, traits::SIMDTrait<ISA>::grainsize>(
            dstc, srcc, n, flag);

    return dst;
}

template <SIMD ISA>
VSMC_STRONG_INLINE void *memmove_simd_front (
        void *dst, const void *src, std::size_t n)
{
    if (dst == src)
        return dst;

    if (n < traits::SIMDTrait<ISA>::alignment) {
        cpy_front_0<ISA>(dst, src, n);
        return dst;
    }

    unsigned flag = cstring_is_aligned<ISA>(dst);
    flag |= cstring_is_aligned<ISA>(src) << 1;
    if (n <= traits::SIMDTrait<ISA>::alignment *
            traits::SIMDTrait<ISA>::grainsize) {
        cpy_front_n_switch<ISA, traits::SIMDTrait<ISA>::grainsize>(
                dst, src, n, flag);
        return dst;
    }

    char *dstc = static_cast<char *>(dst);
    const char *srcc = static_cast<const char *>(src);
    std::size_t offset = reinterpret_cast<uintptr_t>(dstc) % 64;
    if (offset != 0) {
        offset = 64 - offset;
        cpy_front_n_switch<ISA, 64 / traits::SIMDTrait<ISA>::alignment>(
                dstc, srcc, offset, flag);
        n -= offset;
        dstc += offset;
        srcc += offset;
    }
    flag =
        reinterpret_cast<uintptr_t>(src) -
        reinterpret_cast<uintptr_t>(dst) <
        traits::SIMDTrait<ISA>::alignment *
        traits::SIMDTrait<ISA>::grainsize ? 0 : 1;
    flag &= CStringNonTemporalThreshold::instance().over(n);
    flag |= (cstring_is_aligned<ISA>(srcc) << 1) | 2;
    cpy_front_l_switch<ISA, traits::SIMDTrait<ISA>::grainsize>(
            dstc, srcc, n, flag);

    return dst;
}

template <SIMD ISA>
VSMC_STRONG_INLINE void *memmove_simd_back (
        void *dst, const void *src, std::size_t n)
{
    if (dst == src)
        return dst;

    char *dstc = static_cast<char *>(dst) + n;
    const char *srcc = static_cast<const char *>(src) + n;
    if (n < traits::SIMDTrait<ISA>::alignment) {
        cpy_back_0<ISA>(dstc, srcc, n);
        return dst;
    }

    unsigned flag = cstring_is_aligned<ISA>(dstc);
    flag |= cstring_is_aligned<ISA>(srcc) << 1;
    if (n <= traits::SIMDTrait<ISA>::alignment *
            traits::SIMDTrait<ISA>::grainsize) {
        cpy_back_n_switch<ISA, traits::SIMDTrait<ISA>::grainsize>(
                dstc, srcc, n, flag);
        return dst;
    }

    std::size_t offset = reinterpret_cast<uintptr_t>(dstc) % 64;
    if (offset != 0) {
        cpy_back_n_switch<ISA, 32 / traits::SIMDTrait<ISA>::alignment>(
                dstc, srcc, offset, flag);
        n -= offset;
        dstc -= offset;
        srcc -= offset;
    }
    flag =
        reinterpret_cast<uintptr_t>(dst) -
        reinterpret_cast<uintptr_t>(src) <
        traits::SIMDTrait<ISA>::alignment *
        traits::SIMDTrait<ISA>::grainsize ? 0 : 1;
    flag &= CStringNonTemporalThreshold::instance().over(n);
    flag |= (cstring_is_aligned<ISA>(srcc) << 1) | 2;
    cpy_back_l_switch<ISA, traits::SIMDTrait<ISA>::grainsize>(
            dstc, srcc, n, flag);

    return dst;
}

template <SIMD ISA>
VSMC_STRONG_INLINE void *memmove_simd (
        void *dst, const void *src, std::size_t n)
{
    return
        reinterpret_cast<uintptr_t>(dst) <
        reinterpret_cast<uintptr_t>(src) ?
        memmove_simd_front<ISA>(dst, src, n):
        memmove_simd_back<ISA>(dst, src, n);
}

} // namespace vsmc::internal

/// \brief Direct call to `std::memset`
/// \ingroup CString
VSMC_STRONG_INLINE void *memset_std (void *dst, int ch, std::size_t n)
{return std::memset(dst, ch, n);}

/// \brief Direct call to `std::memcpy`
/// \ingroup CString
VSMC_STRONG_INLINE void *memcpy_std (
        void *dst, const void *src, std::size_t n)
{return std::memcpy(dst, src, n);}

/// \brief Direct call to `std::memmove`
/// \ingroup CString
VSMC_STRONG_INLINE void *memmove_std (
        void *dst, const void *src, std::size_t n)
{return std::memmove(dst, src, n);}

#if VSMC_HAS_SSE2

/// \brief SSE2 optimized `memset` with non-temporal store for large buffers
/// \ingroup CString
inline void *memset_sse2 (void *dst, int ch, std::size_t n)
{return internal::memset_simd<SSE2>(dst, ch, n);}

/// \brief SSE2 optimized `memcpy` with non-temporal store for large buffers
/// \ingroup CString
inline void *memcpy_sse2 (void *dst, const void *src, std::size_t n)
{return internal::memcpy_simd<SSE2>(dst, src, n);}

/// \brief SSE2 optimized `memmove` with non-temporal store for large buffers
/// \ingroup CString
inline void *memmove_sse2 (void *dst, const void *src, std::size_t n)
{return internal::memmove_simd<SSE2>(dst, src, n);}

#endif // VSMC_HAS_SSE2

#if VSMC_HAS_AVX

/// \brief AVX optimized `memset` with non-temporal store for large buffers
/// \ingroup CString
inline void *memset_avx (void *dst, int ch, std::size_t n)
{return internal::memset_simd<AVX>(dst, ch, n);}

/// \brief AVX optimized `memcpy` with non-temporal store for large buffers
/// \ingroup CString
inline void *memcpy_avx (void *dst, const void *src, std::size_t n)
{return internal::memcpy_simd<AVX>(dst, src, n);}

/// \brief AVX optimized `memmove` with non-temporal store for large buffers
/// \ingroup CString
inline void *memmove_avx (void *dst, const void *src, std::size_t n)
{return internal::memmove_simd<AVX>(dst, src, n);}

#endif // VSMC_HAS_AVX

namespace internal {

class CStringRuntimeDispatch
{
    public :

    static CStringRuntimeDispatch &instance ()
    {
        static CStringRuntimeDispatch dispatch;

        return dispatch;
    }

    void *memset (void *dst, int ch, std::size_t n) const
    {return memset_(dst, ch, n);}

    void *memcpy (void *dst, const void *src, std::size_t n) const
    {return memcpy_(dst, src, n);}

    void *memmove (void *dst, const void *src, std::size_t n) const
    {return memmove_(dst, src, n);}

    private :

    void *(*memset_) (void *, int, std::size_t);
    void *(*memcpy_) (void *, const void *, std::size_t);
    void *(*memmove_) (void *, const void *, std::size_t);

    CStringRuntimeDispatch ()
    {
        memcpy_ = memcpy_std;
        memmove_ = memmove_std;

#if VSMC_HAS_SSE2
        if (CPUID::has_feature<CPUIDFeatureSSE2>()) {
            memset_ = ::vsmc::memset_sse2;
            memcpy_ = ::vsmc::memcpy_sse2;
            memmove_ = ::vsmc::memmove_sse2;
        }
#endif // VSMC_HAS_SSE2

#if VSMC_HAS_AVX
        if (CPUID::has_feature<CPUIDFeatureAVX>()) {
            memset_ = ::vsmc::memset_avx;
            memcpy_ = ::vsmc::memcpy_avx;
            memmove_ = ::vsmc::memmove_avx;

            return;
        }
#endif // VSMC_HAS_AVX
    }

    CStringRuntimeDispatch (const CStringRuntimeDispatch &);

    CStringRuntimeDispatch &operator= (const CStringRuntimeDispatch &);
}; // class CStringRuntimeDispatc

} // namespace vsmc::internal

/// \brief SIMD optimized `memset` with non-temporal store for large buffers
/// \ingroup CString
inline void *memset (void *dst, int ch, std::size_t n)
{
#if VSMC_CSTRING_RUNTIME_DISPATCH
    return internal::CStringRuntimeDispatch::instance().memset(dst, ch, n);
#else
#if VSMC_HAS_AVX
    return memset_avx(dst, ch, n);
#elif VSMC_HAS_SSE2
    return memset_sse2(dst, ch, n);
#else
    return memset_std(dst, ch, n);
#endif
#endif // VSMC_CSTRING_RUNTIME_DISPATCH
}

/// \brief SIMD optimized `memcpy` with non-temporal store for large buffers
/// \ingroup CString
inline void *memcpy (void *dst, const void *src, std::size_t n)
{
    VSMC_RUNTIME_ASSERT_UTILITY_CSTRING_MEMCPY(dst, src,n);
#if VSMC_CSTRING_RUNTIME_DISPATCH
    return internal::CStringRuntimeDispatch::instance().memcpy(dst, src, n);
#else
#if VSMC_HAS_AVX
    return memcpy_avx(dst, src, n);
#elif VSMC_HAS_SSE2
    return memcpy_sse2(dst, src, n);
#else
    return memcpy_std(dst, src, n);
#endif
#endif // VSMC_CSTRING_RUNTIME_DISPATCH
}

/// \brief SIMD optimized `memmove` with non-temporal store for large buffers
/// \ingroup CString
inline void *memmove (void *dst, const void *src, std::size_t n)
{
#if VSMC_CSTRING_RUNTIME_DISPATCH
    return internal::CStringRuntimeDispatch::instance().memmove(dst, src, n);
#else
#if VSMC_HAS_AVX
    return memmove_avx(dst, src, n);
#elif VSMC_HAS_SSE2
    return memmove_sse2(dst, src, n);
#else
    return memmove_std(dst, src, n);
#endif
#endif // VSMC_CSTRING_RUNTIME_DISPATCH
}

} // namespace vsmc

#endif // VSMC_UTILITY_CSTRING_HPP
