//============================================================================
// include/vsmc/utility/cstring.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distributed under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

/// \addtogroup CString
///
/// This module implement the `memcpy`, etc., function in the `vsmc` namespace.
/// The implementaions are optimzied with SIMD instructions. Fource groups of
/// functions are provided.
///
/// - `memcpy_std` etc., they simply call `std::memcpy` etc.
/// - `memcpy_sse2` etc., they are avialable if at least SSE2 is supported and
/// are optimized with SSE2 instructions
/// - `memcpy_avx` etc., they are avialable if at least AVX is supported and
///
/// are optimized with AVX instructions
/// There are also generic `vsmc::memcpy` etc. They dispatch the call based on
/// the following rules.
///
/// - If AVX is available, then call `memcpy_avx` etc.
/// - Else if SSE2 is available, then call `memcpy_sse2` etc.
/// - Else call `memcpy_std`.
///
/// This dispatch can be done at compile time if the configuration macro
/// `VSMC_CSTRING_RUNTIME_DISPATCH` is zero. This will possibly allow better
/// inlining opportunity. If the macro is non-zero, then it will be done at
/// runtime using `CPUID` information.
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
/// fast as or slightly faster than system library.
/// - For small buffer size (< 1KB), `vsmc::memcpy` etc., simply call
/// `std::memcpy`. The overhead is minimal.
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

/// \brief Shall functions in this module do runtime dispatch
/// \ingroup Config
#ifndef VSMC_CSTRING_RUNTIME_DISPACH
#define VSMC_CSTRING_RUNTIME_DISPACH 0
#endif

/// \brief Threshold above which non-temporal copy shall be used (0 for auto)
/// \ingroup Config
#ifndef VSMC_CSTRING_NON_TEMPORAL_THRESHOLD
#define VSMC_CSTRING_NON_TEMPORAL_THRESHOLD 0
#endif

/// \brief Threshold below which `std::memset` will be called
#ifndef VSMC_CSTRING_STD_MEMSET_THRESHOLD
#define VSMC_CSTRING_STD_MEMSET_THRESHOLD 1024
#endif

/// \brief Threshold below which `std::memcpy` will be called
#ifndef VSMC_CSTRING_STD_MEMCPY_THRESHOLD
#define VSMC_CSTRING_STD_MEMCPY_THRESHOLD 1024
#endif

/// \brief Threshold below which `std::memmove` will be called
#ifndef VSMC_CSTRING_STD_MEMMOVE_THRESHOLD
#define VSMC_CSTRING_STD_MEMMOVE_THRESHOLD 1024
#endif

#ifdef __SSE2__
#include <emmintrin.h>
#endif // __SSE2__

#ifdef __AVX__
#include <immintrin.h>
#endif // __AVX__

#define VSMC_RUNTIME_ASSERT_UTILITY_CSTRING_FLAG(func) \
    VSMC_RUNTIME_ASSERT(false, ("**vsmc::internal::"#func" UNDEFINED FLAG"))

#define VSMC_DEFINE_UTILITY_CSTRING_SET_SIMD_8(\
        da, simd, align, c, m, cs, s1, s) \
template <>                                                                  \
inline void set_##simd##_8<da> (void *dst, int ch, std::size_t n)            \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    m m0 = cs(s1(static_cast<char>(static_cast<unsigned char>(ch))));        \
                                                                             \
    if (n >= align * 4) {                                                    \
        n -= align * 4;                                                      \
        s(dstc + align * 0 / sizeof(c), m0);                                 \
        s(dstc + align * 1 / sizeof(c), m0);                                 \
        s(dstc + align * 2 / sizeof(c), m0);                                 \
        s(dstc + align * 3 / sizeof(c), m0);                                 \
        dstc += align * 4 / sizeof(c);                                       \
    }                                                                        \
    if (n >= align * 2) {                                                    \
        n -= align * 2;                                                      \
        s(dstc + align * 0 / sizeof(c), m0);                                 \
        s(dstc + align * 1 / sizeof(c), m0);                                 \
        dstc += align * 2 / sizeof(c);                                       \
    }                                                                        \
    if (n >= align * 1) {                                                    \
        n -= align * 1;                                                      \
        s(dstc + align * 0 / sizeof(c), m0);                                 \
        dstc += align * 1 / sizeof(c);                                       \
    }                                                                        \
    set_1(dstc, ch, n);                                                      \
}

#define VSMC_DEFINE_UTILITY_CSTRING_SET_SIMD_8_FLAG(simd) \
inline void set_##simd##_8 (void *dst, int ch, std::size_t n, unsigned flag) \
{                                                                            \
    flag == 0 ?                                                              \
        set_##simd##_8<false>(dst, ch, n):                                   \
        set_##simd##_8<true >(dst, ch, n);                                   \
}

#define VSMC_DEFINE_UTILITY_CSTRING_SET_SIMD(\
        da, nt, simd, align, c, m, cs, s1, s) \
template <>                                                                  \
inline void set_##simd<da, nt> (void *dst, int ch, std::size_t n)            \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    m m0 = cs(s1(static_cast<char>(static_cast<unsigned char>(ch))));        \
                                                                             \
    std::size_t nm = n / (align * 8);                                        \
    for (std::size_t i = 0; i != nm; ++i) {                                  \
        s(dstc + align * 0 / sizeof(c), m0);                                 \
        s(dstc + align * 1 / sizeof(c), m0);                                 \
        s(dstc + align * 2 / sizeof(c), m0);                                 \
        s(dstc + align * 3 / sizeof(c), m0);                                 \
        s(dstc + align * 4 / sizeof(c), m0);                                 \
        s(dstc + align * 5 / sizeof(c), m0);                                 \
        s(dstc + align * 6 / sizeof(c), m0);                                 \
        s(dstc + align * 7 / sizeof(c), m0);                                 \
        dstc += align * 8 / sizeof(c);                                       \
    }                                                                        \
    set_##simd##_8<da>(dstc, ch, n % (align * 8));                           \
}

#define VSMC_DEFINE_UTILITY_CSTRING_SET_SIMD_FLAG(simd) \
inline void set_##simd (void *dst, int ch, std::size_t n, unsigned flag)     \
{                                                                            \
    switch (flag) {                                                          \
        case 0: set_##simd<false, false>(dst, ch, n); break;                 \
        case 2: set_##simd<true,  false>(dst, ch, n); break;                 \
        case 3: set_##simd<true,  true >(dst, ch, n); break;                 \
        default :                                                            \
            VSMC_RUNTIME_ASSERT_UTILITY_CSTRING_FLAG(set_##simd);            \
            break;                                                           \
    }                                                                        \
}

#define VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD_8(\
        sa, da, simd, align, c, m, l, s) \
template <>                                                                  \
inline void cpy_front_##simd##_8<sa, da> (                                   \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    const c *srcc = static_cast<const c *>(src);                             \
                                                                             \
    if (n >= align * 4) {                                                    \
        n -= align * 4;                                                      \
        m m0 = l(srcc + align * 0 / sizeof(c));                              \
        m m1 = l(srcc + align * 1 / sizeof(c));                              \
        m m2 = l(srcc + align * 2 / sizeof(c));                              \
        m m3 = l(srcc + align * 3 / sizeof(c));                              \
        s(dstc + align * 0 / sizeof(c), m0);                                 \
        s(dstc + align * 1 / sizeof(c), m1);                                 \
        s(dstc + align * 2 / sizeof(c), m2);                                 \
        s(dstc + align * 3 / sizeof(c), m3);                                 \
        dstc += align * 4 / sizeof(c);                                       \
        srcc += align * 4 / sizeof(c);                                       \
    }                                                                        \
    if (n >= align * 2) {                                                    \
        n -= align * 2;                                                      \
        m m0 = l(srcc + align * 0 / sizeof(c));                              \
        m m1 = l(srcc + align * 1 / sizeof(c));                              \
        s(dstc + align * 0 / sizeof(c), m0);                                 \
        s(dstc + align * 1 / sizeof(c), m1);                                 \
        dstc += align * 2 / sizeof(c);                                       \
        srcc += align * 2 / sizeof(c);                                       \
    }                                                                        \
    if (n >= align * 1) {                                                    \
        n -= align * 1;                                                      \
        m m0 = l(srcc + align * 0 / sizeof(c));                              \
        s(dstc + align * 0 / sizeof(c), m0);                                 \
        dstc += align * 1 / sizeof(c);                                       \
        srcc += align * 1 / sizeof(c);                                       \
    }                                                                        \
    cpy_front_1(dstc, srcc, n);                                              \
}                                                                            \
                                                                             \
template <>                                                                  \
inline void cpy_back_##simd##_8<sa, da> (                                    \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    const c *srcc = static_cast<const c *>(src);                             \
                                                                             \
    if (n >= align * 4) {                                                    \
        n -= align * 4;                                                      \
        m m1 = l(srcc - align * 1 / sizeof(c));                              \
        m m2 = l(srcc - align * 2 / sizeof(c));                              \
        m m3 = l(srcc - align * 3 / sizeof(c));                              \
        m m4 = l(srcc - align * 4 / sizeof(c));                              \
        s(dstc - align * 1 / sizeof(c), m1);                                 \
        s(dstc - align * 2 / sizeof(c), m2);                                 \
        s(dstc - align * 3 / sizeof(c), m3);                                 \
        s(dstc - align * 4 / sizeof(c), m4);                                 \
        dstc -= align * 4 / sizeof(c);                                       \
        srcc -= align * 4 / sizeof(c);                                       \
    }                                                                        \
    if (n >= align * 2) {                                                    \
        n -= align * 2;                                                      \
        m m1 = l(srcc - align * 1 / sizeof(c));                              \
        m m2 = l(srcc - align * 2 / sizeof(c));                              \
        s(dstc - align * 1 / sizeof(c), m1);                                 \
        s(dstc - align * 2 / sizeof(c), m2);                                 \
        dstc -= align * 2 / sizeof(c);                                       \
        srcc -= align * 2 / sizeof(c);                                       \
    }                                                                        \
    if (n >= align * 1) {                                                    \
        n -= align * 1;                                                      \
        m m1 = l(srcc - align * 1 / sizeof(c));                              \
        s(dstc - align * 1 / sizeof(c), m1);                                 \
        dstc -= align * 1 / sizeof(c);                                       \
        srcc -= align * 1 / sizeof(c);                                       \
    }                                                                        \
    cpy_back_1(dstc, srcc, n);                                               \
}

#define VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD_8_FLAG(simd) \
inline void cpy_front_##simd##_8 (void *dst, const void *src, std::size_t n, \
        unsigned flag)                                                       \
{                                                                            \
    switch (flag) {                                                          \
        case 0 : cpy_front_##simd##_8<false, false>(dst, src, n); break;     \
        case 1 : cpy_front_##simd##_8<false, true >(dst, src, n); break;     \
        case 2 : cpy_front_##simd##_8<true,  false>(dst, src, n); break;     \
        case 3 : cpy_front_##simd##_8<true,  true >(dst, src, n); break;     \
        default :                                                            \
            VSMC_RUNTIME_ASSERT_UTILITY_CSTRING_FLAG(cpy_front_##simd##_8);  \
            break;                                                           \
    }                                                                        \
}                                                                            \
                                                                             \
inline void cpy_back_##simd##_8 (void *dst, const void *src, std::size_t n,  \
        unsigned flag)                                                       \
{                                                                            \
    switch (flag) {                                                          \
        case 0 : cpy_back_##simd##_8<false, false>(dst, src, n); break;      \
        case 1 : cpy_back_##simd##_8<false, true >(dst, src, n); break;      \
        case 2 : cpy_back_##simd##_8<true,  false>(dst, src, n); break;      \
        case 3 : cpy_back_##simd##_8<true,  true >(dst, src, n); break;      \
        default :                                                            \
            VSMC_RUNTIME_ASSERT_UTILITY_CSTRING_FLAG(cpy_back_##simd##_8);   \
            break;                                                           \
    }                                                                        \
}

#define VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD(\
        sa, da, nt, simd, align, c, m, l, s) \
template <>                                                                  \
inline void cpy_front_##simd<sa, da, nt> (                                   \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    const c *srcc = static_cast<const c *>(src);                             \
                                                                             \
    std::size_t nm = n / (align * 8);                                        \
    for (std::size_t i = 0; i != nm; ++i) {                                  \
        m m0 = l(srcc + align * 0 / sizeof(c));                              \
        m m1 = l(srcc + align * 1 / sizeof(c));                              \
        m m2 = l(srcc + align * 2 / sizeof(c));                              \
        m m3 = l(srcc + align * 3 / sizeof(c));                              \
        m m4 = l(srcc + align * 4 / sizeof(c));                              \
        m m5 = l(srcc + align * 5 / sizeof(c));                              \
        m m6 = l(srcc + align * 6 / sizeof(c));                              \
        m m7 = l(srcc + align * 7 / sizeof(c));                              \
        s(dstc + align * 0 / sizeof(c), m0);                                 \
        s(dstc + align * 1 / sizeof(c), m1);                                 \
        s(dstc + align * 2 / sizeof(c), m2);                                 \
        s(dstc + align * 3 / sizeof(c), m3);                                 \
        s(dstc + align * 4 / sizeof(c), m4);                                 \
        s(dstc + align * 5 / sizeof(c), m5);                                 \
        s(dstc + align * 6 / sizeof(c), m6);                                 \
        s(dstc + align * 7 / sizeof(c), m7);                                 \
        dstc += align * 8 / sizeof(c);                                       \
        srcc += align * 8 / sizeof(c);                                       \
    }                                                                        \
    cpy_front_##simd##_8<sa, da>(dstc, srcc, n % (align * 8));               \
}                                                                            \
                                                                             \
template <>                                                                  \
inline void cpy_back_##simd<sa, da, nt> (                                    \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    if (n == 0)                                                              \
        return;                                                              \
                                                                             \
    c *dstc = static_cast<c *>(dst);                                         \
    const c *srcc = static_cast<const c *>(src);                             \
                                                                             \
    std::size_t nm = n / (align * 8);                                        \
    for (std::size_t i = 0; i != nm; ++i) {                                  \
        m m1 = l(srcc - align * 1 / sizeof(c));                              \
        m m2 = l(srcc - align * 2 / sizeof(c));                              \
        m m3 = l(srcc - align * 3 / sizeof(c));                              \
        m m4 = l(srcc - align * 4 / sizeof(c));                              \
        m m5 = l(srcc - align * 5 / sizeof(c));                              \
        m m6 = l(srcc - align * 6 / sizeof(c));                              \
        m m7 = l(srcc - align * 7 / sizeof(c));                              \
        m m8 = l(srcc - align * 8 / sizeof(c));                              \
        s(dstc - align * 1 / sizeof(c), m1);                                 \
        s(dstc - align * 2 / sizeof(c), m2);                                 \
        s(dstc - align * 3 / sizeof(c), m3);                                 \
        s(dstc - align * 4 / sizeof(c), m4);                                 \
        s(dstc - align * 5 / sizeof(c), m5);                                 \
        s(dstc - align * 6 / sizeof(c), m6);                                 \
        s(dstc - align * 7 / sizeof(c), m7);                                 \
        s(dstc - align * 8 / sizeof(c), m8);                                 \
        dstc -= align * 8 / sizeof(c);                                       \
        srcc -= align * 8 / sizeof(c);                                       \
    }                                                                        \
    cpy_back_##simd##_8<sa, da>(dstc, srcc, n % (align * 8));                \
}

#define VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD_FLAG(simd) \
inline void cpy_front_##simd (void *dst, const void *src, std::size_t n,     \
        unsigned flag)                                                       \
{                                                                            \
    switch (flag) {                                                          \
        case 0 : cpy_front_##simd<false, false, false>(dst, src, n); break;  \
        case 2 : cpy_front_##simd<false, true,  false>(dst, src, n); break;  \
        case 3 : cpy_front_##simd<false, true,  true >(dst, src, n); break;  \
        case 4 : cpy_front_##simd<true,  false, false>(dst, src, n); break;  \
        case 6 : cpy_front_##simd<true,  true,  false>(dst, src, n); break;  \
        case 7 : cpy_front_##simd<true,  true,  true >(dst, src, n); break;  \
        default :                                                            \
            VSMC_RUNTIME_ASSERT_UTILITY_CSTRING_FLAG(cpy_front_##simd);      \
            break;                                                           \
    }                                                                        \
}                                                                            \
                                                                             \
inline void cpy_back_##simd (void *dst, const void *src, std::size_t n,      \
        unsigned flag)                                                       \
{                                                                            \
    switch (flag) {                                                          \
        case 0 : cpy_back_##simd<false, false, false>(dst, src, n); break;   \
        case 2 : cpy_back_##simd<false, true,  false>(dst, src, n); break;   \
        case 3 : cpy_back_##simd<false, true,  true >(dst, src, n); break;   \
        case 4 : cpy_back_##simd<true,  false, false>(dst, src, n); break;   \
        case 6 : cpy_back_##simd<true,  true,  false>(dst, src, n); break;   \
        case 7 : cpy_back_##simd<true,  true,  true >(dst, src, n); break;   \
        default :                                                            \
            VSMC_RUNTIME_ASSERT_UTILITY_CSTRING_FLAG(cpy_back_##simd);       \
            break;                                                           \
    }                                                                        \
}

#define VSMC_DEFINE_UTILITY_CSTRING_MEMSET(simd, align) \
inline void *memset_##simd (void *dst, int ch, std::size_t n)                \
{                                                                            \
    char *dstc = static_cast<char *>(dst);                                   \
                                                                             \
    unsigned flag = internal::cstring_is_aligned<align>(dstc);               \
    if (n < align * 8) {                                                     \
        internal::set_##simd##_8(dstc, ch, n, flag);                         \
        return dst;                                                          \
    }                                                                        \
                                                                             \
    std::size_t offset = reinterpret_cast<uintptr_t>(dstc) % (align * 8);    \
    if (offset != 0) {                                                       \
        offset = (align * 8) - offset;                                       \
        internal::set_##simd##_8(dstc, ch, offset, flag);                    \
        n -= offset;                                                         \
        dstc += offset;                                                      \
    }                                                                        \
                                                                             \
    flag = CStringNonTemporalThreshold::instance().over(n) | 2;              \
    internal::set_##simd(dstc, ch, n, flag);                                 \
                                                                             \
    return dst;                                                              \
}

#define VSMC_DEFINE_UTILITY_CSTRING_MEMCPY(simd, align) \
inline void *memcpy_##simd (void *dst, const void *src, std::size_t n)       \
{                                                                            \
    if (dst == src)                                                          \
        return dst;                                                          \
                                                                             \
    char *dstc = static_cast<char *>(dst);                                   \
    const char *srcc = static_cast<const char *>(src);                       \
                                                                             \
    unsigned flag = internal::cstring_is_aligned<align>(dstc) >> 1;          \
    flag |= internal::cstring_is_aligned<align>(srcc);                       \
                                                                             \
    if (n < align * 8) {                                                     \
        internal::cpy_front_##simd##_8(dstc, srcc, n, flag);                 \
        return dst;                                                          \
    }                                                                        \
                                                                             \
    std::size_t offset = reinterpret_cast<uintptr_t>(dstc) % (align * 8);    \
    if (offset != 0) {                                                       \
        offset = (align * 8) - offset;                                       \
        internal::cpy_front_##simd##_8(dstc, srcc, offset, flag);            \
        n -= offset;                                                         \
        dstc += offset;                                                      \
        srcc += offset;                                                      \
    }                                                                        \
                                                                             \
    flag = CStringNonTemporalThreshold::instance().over(n) | 2;              \
    flag |= internal::cstring_is_aligned<align>(srcc) << 1;                  \
    internal::cpy_front_##simd(dstc, srcc, n, flag);                         \
                                                                             \
    return dst;                                                              \
}

#define VSMC_DEFINE_UTILITY_CSTRING_MEMMOVE(simd, align) \
inline void *memmove_##simd (void *dst, const void *src, std::size_t n)      \
{                                                                            \
    if (dst == src)                                                          \
        return dst;                                                          \
                                                                             \
    uintptr_t dsta = reinterpret_cast<uintptr_t>(dst);                       \
    uintptr_t srca = reinterpret_cast<uintptr_t>(src);                       \
    if (dsta < srca)                                                         \
        return memcpy_##simd(dst, src, n);                                   \
                                                                             \
    char *dstc = static_cast<char *>(dst) + n;                               \
    const char *srcc = static_cast<const char *>(src) + n;                   \
                                                                             \
    unsigned flag = internal::cstring_is_aligned<align>(dstc) >> 1;          \
    flag |= internal::cstring_is_aligned<align>(srcc);                       \
                                                                             \
    if (n < align * 8) {                                                     \
        internal::cpy_back_##simd##_8(dstc, srcc, n, flag);                  \
        return dst;                                                          \
    }                                                                        \
                                                                             \
    std::size_t offset = reinterpret_cast<uintptr_t>(dstc) % (align * 8);    \
    if (offset != 0) {                                                       \
        internal::cpy_back_##simd##_8(dstc, srcc, offset, flag);             \
        n -= offset;                                                         \
        dstc -= offset;                                                      \
        srcc -= offset;                                                      \
    }                                                                        \
                                                                             \
    flag = CStringNonTemporalThreshold::instance().over(n) | 2;              \
    flag |= internal::cstring_is_aligned<align>(srcc) << 1;                  \
    internal::cpy_back_##simd(dstc, srcc, n, flag);                          \
                                                                             \
    return dst;                                                              \
}

namespace vsmc {

namespace internal {

template <std::size_t Alignment>
inline unsigned cstring_is_aligned (const void *ptr)
{return reinterpret_cast<uintptr_t>(ptr) % Alignment == 0 ? 2 : 0;}

inline void set_1 (void *dst, int ch, std::size_t n)
{
    if (n != 0)
        std::memset(dst, ch, n);
}

inline void cpy_front_1 (void *dst, const void *src, std::size_t n)
{
    if (n != 0)
        std::memmove(dst, src, n);
}

inline void cpy_back_1 (void *dst, const void *src, std::size_t n)
{
    if (n != 0) {
        dst = static_cast<void *>(static_cast<char *>(dst) - n);
        src = static_cast<const void *>(static_cast<const char *>(src) - n);
        std::memmove(dst, src, n);
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
            if (level <= 3 && level > cache_level) {
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

    CStringNonTemporalThreshold () :
        threshold_(VSMC_CSTRING_NON_TEMPORAL_THRESHOLD)
    {if (threshold_ == 0) set();}

    CStringNonTemporalThreshold (const CStringNonTemporalThreshold &);

    CStringNonTemporalThreshold &operator= (
            const CStringNonTemporalThreshold &);
}; // class CStringNonTemporalThreshold

/// \brief Direct call to `std::memset`
/// \ingroup CString
inline void *memset_std (void *dst, int ch, std::size_t n)
{return std::memset(dst, ch, n);}

/// \brief Direct call to `std::memcpy`
/// \ingroup CString
inline void *memcpy_std (void *dst, const void *src, std::size_t n)
{return std::memcpy(dst, src, n);}

/// \brief Direct call to `std::memmove`
/// \ingroup CString
inline void *memmove_std (void *dst, const void *src, std::size_t n)
{return std::memmove(dst, src, n);}

#ifdef __SSE2__

namespace internal {

template <bool>
inline void set_sse2_8 (void *, int, std::size_t);

VSMC_DEFINE_UTILITY_CSTRING_SET_SIMD_8(false, sse2, 16,
        double, __m128d, _mm_castsi128_pd, _mm_set1_epi8,
        _mm_storeu_pd)
VSMC_DEFINE_UTILITY_CSTRING_SET_SIMD_8(true, sse2, 16,
        double, __m128d, _mm_castsi128_pd, _mm_set1_epi8,
        _mm_store_pd)

VSMC_DEFINE_UTILITY_CSTRING_SET_SIMD_8_FLAG(sse2)

template <bool, bool>
inline void set_sse2 (void *, int, std::size_t);

VSMC_DEFINE_UTILITY_CSTRING_SET_SIMD(false, false, sse2, 16,
        double, __m128d, _mm_castsi128_pd, _mm_set1_epi8,
        _mm_storeu_pd)
VSMC_DEFINE_UTILITY_CSTRING_SET_SIMD(true, false, sse2, 16,
        double, __m128d, _mm_castsi128_pd, _mm_set1_epi8,
        _mm_store_pd)
VSMC_DEFINE_UTILITY_CSTRING_SET_SIMD(true, true, sse2, 16,
        double, __m128d, _mm_castsi128_pd, _mm_set1_epi8,
        _mm_stream_pd)

VSMC_DEFINE_UTILITY_CSTRING_SET_SIMD_FLAG(sse2)

template <bool, bool>
inline void cpy_front_sse2_8 (void *, const void *, std::size_t);

template <bool, bool>
inline void cpy_back_sse2_8 (void *, const void *, std::size_t);

VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD_8(false, false, sse2, 16,
        double, __m128d, _mm_loadu_pd, _mm_storeu_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD_8(false, true, sse2, 16,
        double, __m128d, _mm_loadu_pd, _mm_store_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD_8(true, false, sse2, 16,
        double, __m128d, _mm_load_pd, _mm_storeu_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD_8(true, true, sse2, 16,
        double, __m128d, _mm_load_pd, _mm_store_pd)

VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD_8_FLAG(sse2)

template <bool, bool, bool>
inline void cpy_front_sse2 (void *, const void *, std::size_t);

template <bool, bool, bool>
inline void cpy_back_sse2 (void *, const void *, std::size_t);

VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD(false, false, false, sse2, 16,
        double, __m128d, _mm_loadu_pd, _mm_storeu_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD(false, true, false, sse2, 16,
        double, __m128d, _mm_loadu_pd, _mm_store_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD(false, true, true, sse2, 16,
        double, __m128d, _mm_loadu_pd, _mm_stream_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD(true, false, false, sse2, 16,
        double, __m128d, _mm_load_pd, _mm_storeu_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD(true, true, false, sse2, 16,
        double, __m128d, _mm_load_pd, _mm_store_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD(true, true, true, sse2, 16,
        double, __m128d, _mm_load_pd, _mm_stream_pd)

VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD_FLAG(sse2)

} // namespace vsmc::internal

/// \brief SSE2 optimized `memeset` with non-temporal store for large buffers
/// \ingroup CString
VSMC_DEFINE_UTILITY_CSTRING_MEMSET(sse2, 16)

/// \brief SSE2 optimized `memcpy` with non-temporal store for large buffers
/// \ingroup CString
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY(sse2, 16)

/// \brief SSE2 optimized `memmove` with non-temporal store for large buffers
/// \ingroup CString
VSMC_DEFINE_UTILITY_CSTRING_MEMMOVE(sse2, 16)

#endif // __SSE2__

#ifdef __AVX__

namespace internal {

template <bool>
inline void set_avx_8 (void *, int, std::size_t);

VSMC_DEFINE_UTILITY_CSTRING_SET_SIMD_8(false, avx, 32,
        double, __m256d, _mm256_castsi256_pd, _mm256_set1_epi8,
        _mm256_storeu_pd)
VSMC_DEFINE_UTILITY_CSTRING_SET_SIMD_8(true, avx, 32,
        double, __m256d, _mm256_castsi256_pd, _mm256_set1_epi8,
        _mm256_store_pd)

VSMC_DEFINE_UTILITY_CSTRING_SET_SIMD_8_FLAG(avx)

template <bool, bool>
inline void set_avx (void *, int, std::size_t);

VSMC_DEFINE_UTILITY_CSTRING_SET_SIMD(false, false, avx, 32,
        double, __m256d, _mm256_castsi256_pd, _mm256_set1_epi8,
        _mm256_storeu_pd)
VSMC_DEFINE_UTILITY_CSTRING_SET_SIMD(true, false, avx, 32,
        double, __m256d, _mm256_castsi256_pd, _mm256_set1_epi8,
        _mm256_store_pd)
VSMC_DEFINE_UTILITY_CSTRING_SET_SIMD(true, true, avx, 32,
        double, __m256d, _mm256_castsi256_pd, _mm256_set1_epi8,
        _mm256_stream_pd)

VSMC_DEFINE_UTILITY_CSTRING_SET_SIMD_FLAG(avx)

template <bool, bool>
inline void cpy_front_avx_8 (void *, const void *, std::size_t);

template <bool, bool>
inline void cpy_back_avx_8 (void *, const void *, std::size_t);

VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD_8(false, false, avx, 32,
        double, __m256d, _mm256_loadu_pd, _mm256_storeu_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD_8(false, true, avx, 32,
        double, __m256d, _mm256_loadu_pd, _mm256_store_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD_8(true, false, avx, 32,
        double, __m256d, _mm256_load_pd, _mm256_storeu_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD_8(true, true, avx, 32,
        double, __m256d, _mm256_load_pd, _mm256_store_pd)

VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD_8_FLAG(avx)

template <bool, bool, bool>
inline void cpy_front_avx (void *, const void *, std::size_t);

template <bool, bool, bool>
inline void cpy_back_avx (void *, const void *, std::size_t);

VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD(false, false, false, avx, 32,
        double, __m256d, _mm256_loadu_pd, _mm256_storeu_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD(false, true, false, avx, 32,
        double, __m256d, _mm256_loadu_pd, _mm256_store_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD(false, true, true, avx, 32,
        double, __m256d, _mm256_loadu_pd, _mm256_stream_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD(true, false, false, avx, 32,
        double, __m256d, _mm256_load_pd, _mm256_storeu_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD(true, true, false, avx, 32,
        double, __m256d, _mm256_load_pd, _mm256_store_pd)
VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD(true, true, true, avx, 32,
        double, __m256d, _mm256_load_pd, _mm256_stream_pd)

VSMC_DEFINE_UTILITY_CSTRING_CPY_SIMD_FLAG(avx)

} // namespace vsmc::internal

/// \brief AVX optimized `memeset` with non-temporal store for large buffers
/// \ingroup CString
VSMC_DEFINE_UTILITY_CSTRING_MEMSET(avx, 32)

/// \brief AVX optimized `memcpy` with non-temporal store for large buffers
/// \ingroup CString
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY(avx, 32)

/// \brief AVX optimized `memmove` with non-temporal store for large buffers
/// \ingroup CString
VSMC_DEFINE_UTILITY_CSTRING_MEMMOVE(avx, 32)

#endif // __AVX__

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

#ifdef __SSE2__
        if (CPUID::has_feature<CPUIDFeatureSSE2>()) {
            memset_ = ::vsmc::memset_sse2;
            memcpy_ = ::vsmc::memcpy_sse2;
            memmove_ = ::vsmc::memmove_sse2;
        }
#endif // __SSE2__

#ifdef __AVX__
        if (CPUID::has_feature<CPUIDFeatureAVX>()) {
            memset_ = ::vsmc::memset_avx;
            memcpy_ = ::vsmc::memcpy_avx;
            memmove_ = ::vsmc::memmove_avx;

            return;
        }
#endif // __AVX__
    }

    CStringRuntimeDispatch (const CStringRuntimeDispatch &);

    CStringRuntimeDispatch &operator= (const CStringRuntimeDispatch &);
}; // class CStringRuntimeDispatc

} // namespace vsmc::internal

/// \brief SIMD optimized `memset` with non-temporal store for large buffers
/// \ingroup CString
inline void *memset (void *dst, int ch, std::size_t n)
{
    if (n < VSMC_CSTRING_STD_MEMSET_THRESHOLD)
        return memset_std(dst, ch, n);
#if VSMC_CSTRING_RUNTIME_DISPATCH
    return internal::CStringRuntimeDispatch::instance().memset(dst, ch, n);
#else
#if defined(__AVX__)
    return memset_avx(dst, ch, n);
#elif defined(__SSE2__)
    return memset_sse2(dst, ch, n);
#else
    return memset_std(dst, ch, n);
#endif // defined(__AVX__)
#endif // VSMC_CSTRING_RUNTIME_DISPATCH
}

/// \brief SIMD optimized `memcpy` with non-temporal store for large buffers
/// \ingroup CString
inline void *memcpy (void *dst, const void *src, std::size_t n)
{
    if (n < VSMC_CSTRING_STD_MEMCPY_THRESHOLD)
        return memcpy_std(dst, src, n);
#if VSMC_CSTRING_RUNTIME_DISPATCH
    return internal::CStringRuntimeDispatch::instance().memcpy(dst, src, n);
#else
#if defined(__AVX__)
    return memcpy_avx(dst, src, n);
#elif defined(__SSE2__)
    return memcpy_sse2(dst, src, n);
#else
    return memcpy_std(dst, src, n);
#endif // defined(__AVX__)
#endif // VSMC_CSTRING_RUNTIME_DISPATCH
}

/// \brief SIMD optimized `memmove` with non-temporal store for large buffers
/// \ingroup CString
inline void *memmove (void *dst, const void *src, std::size_t n)
{
    if (n < VSMC_CSTRING_STD_MEMMOVE_THRESHOLD)
        return memmove_std(dst, src, n);
#if VSMC_CSTRING_RUNTIME_DISPATCH
    return internal::CStringRuntimeDispatch::instance().memmove(dst, src, n);
#else
#if defined(__AVX__)
    return memmove_avx(dst, src, n);
#elif defined(__SSE2__)
    return memmove_sse2(dst, src, n);
#else
    return memmove_std(dst, src, n);
#endif // defined(__AVX__)
#endif // VSMC_CSTRING_RUNTIME_DISPATCH
}

} // namespace vsmc

#endif // VSMC_UTILITY_CSTRING_HPP
