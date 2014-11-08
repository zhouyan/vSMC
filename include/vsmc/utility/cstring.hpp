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
/// - `memcpy_sse3` etc., they are avialable if at least SSE3 is supported and
/// are optimized with SSE3 instructions
/// - `memcpy_avx` etc., they are avialable if at least AVX is supported and
///
/// are optimized with AVX instructions
/// There are also generic `vsmc::memcpy` etc. They dispatch the call based on
/// the following rules.
///
/// - If buffer size (in bytes) is smaller than `VSMC_CSTRING_STD_THRESHOLD`,
/// call `memcpy_std` etc.
/// - Else if AVX is available, then call `memcpy_avx` etc.
/// - Else if SSE2 is available, then call `memcpy_sse2` etc.
/// - Else if SSE3 is available, then call `memcpy_sse3` etc.
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

/// \brief Threshold of bytes below which to call standard library functions
/// \ingroup Config
#ifndef VSMC_CSTRING_STD_THRESHOLD
#define VSMC_CSTRING_STD_THRESHOLD (1 << 10)
#endif

#ifdef __SSE2__
#include <emmintrin.h>
#endif // __SSE2__

#ifdef __SSE3__
#include <pmmintrin.h>
#endif // __SSE3__

#ifdef __AVX__
#include <immintrin.h>

#endif // __AVX__

#define VSMC_DEFINE_UTILITY_CSTRING_SIMD(sa, nt, simd, align, c, m, l, s) \
template <>                                                                  \
inline void cpy_front_##simd<sa, nt> (                                       \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    c *dstc = static_cast<c *>(dst);                                         \
    const c *srcc = static_cast<const c *>(src);                             \
                                                                             \
    std::size_t nm = n / (align * 2);                                        \
    for (std::size_t i = 0; i != nm; ++i) {                                  \
        m m1 = l(srcc);                                                      \
        m m2 = l(srcc + (align / sizeof(c)));                                \
        s(dstc, m1);                                                         \
        s(dstc + (align / sizeof(c)), m2);                                   \
        dstc += align / sizeof(c) * 2;                                       \
        srcc += align / sizeof(c) * 2;                                       \
    }                                                                        \
}                                                                            \
                                                                             \
template <>                                                                  \
inline void cpy_back_##simd<sa, nt> (                                        \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    c *dstc = static_cast<c *>(dst);                                         \
    const c *srcc = static_cast<const c *>(src);                             \
                                                                             \
    std::size_t nm = n / (align * 2);                                        \
    for (std::size_t i = 0; i != nm; ++i) {                                  \
        dstc -= align / sizeof(c) * 2;                                       \
        srcc -= align / sizeof(c) * 2;                                       \
        m m1 = l(srcc);                                                      \
        m m2 = l(srcc + (align / sizeof(c)));                                \
        s(dstc, m1);                                                         \
        s(dstc + (align / sizeof(c)), m2);                                   \
    }                                                                        \
}

#define VSMC_DEFINE_UTILITY_CSTRING_MEMCPY(simd, align) \
inline void *memcpy_##simd (void *dst, const void *src, std::size_t n)       \
{                                                                            \
    if (dst == src)                                                          \
        return dst;                                                          \
                                                                             \
    if (n < align * 4)                                                       \
        return memcpy_std(dst, src, n);                                      \
                                                                             \
    char *dstc = static_cast<char *>(dst);                                   \
    const char *srcc = static_cast<const char *>(src);                       \
    std::size_t offset = reinterpret_cast<uintptr_t>(dstc) % align;          \
    if (offset != 0) {                                                       \
        offset = align - offset;                                             \
        dstc += offset;                                                      \
        srcc += offset;                                                      \
        n -= offset;                                                         \
        memcpy_std(dst, src, offset);                                        \
    }                                                                        \
                                                                             \
    unsigned flag = CStringNonTemporalThreshold::instance().over(n);         \
    flag |= internal::cstring_is_aligned<align>(srcc);                       \
    switch (flag) {                                                          \
        case 0 : internal::cpy_front_##simd<false, false>(dstc, srcc, n);    \
                 break;                                                      \
        case 1 : internal::cpy_front_##simd<false, true >(dstc, srcc, n);    \
                 break;                                                      \
        case 2 : internal::cpy_front_##simd<true,  false>(dstc, srcc, n);    \
                 break;                                                      \
        case 3 : internal::cpy_front_##simd<true,  true >(dstc, srcc, n);    \
                 break;                                                      \
        default :                                                            \
                 break;                                                      \
    }                                                                        \
                                                                             \
    std::size_t m = (n / (align * 2)) * (align * 2);                         \
    dstc += m;                                                               \
    srcc += m;                                                               \
    memcpy_std(dstc, srcc, n % (align * 2));                                 \
                                                                             \
    return dst;                                                              \
}

#define VSMC_DEFINE_UTILITY_CSTRING_MEMMOVE(simd, align) \
inline void *memmove_##simd (void *dst, const void *src, std::size_t n)      \
{                                                                            \
    if (dst == src)                                                          \
        return dst;                                                          \
                                                                             \
    if (n < align * 4)                                                       \
        return memmove_std(dst, src, n);                                     \
                                                                             \
    uintptr_t dsta = reinterpret_cast<uintptr_t>(dst);                       \
    uintptr_t srca = reinterpret_cast<uintptr_t>(src);                       \
    if (srca + n <= dsta || dsta + n <= srca)                                \
        return memcpy_sse2(dst, src, n);                                     \
                                                                             \
    if (dsta < srca) {                                                       \
        char *dstc = static_cast<char *>(dst);                               \
        const char *srcc = static_cast<const char *>(src);                   \
        std::size_t offset = reinterpret_cast<uintptr_t>(dstc) % align;      \
        if (offset != 0) {                                                   \
            offset = align - offset;                                         \
            dstc += offset;                                                  \
            srcc += offset;                                                  \
            n -= offset;                                                     \
            memmove_std(dst, src, offset);                                   \
        }                                                                    \
                                                                             \
        unsigned flag = CStringNonTemporalThreshold::instance().over(n);     \
        flag &= CStringNonTemporalThreshold::instance().over(srca - dsta);   \
        flag |= internal::cstring_is_aligned<align>(srcc);                   \
        switch (flag) {                                                      \
            case 0 : internal::cpy_front_##simd<false, false>(dstc, srcc, n);\
                     break;                                                  \
            case 1 : internal::cpy_front_##simd<false,true >(dstc, srcc, n); \
                     break;                                                  \
            case 2 : internal::cpy_front_##simd<true, false>(dstc, srcc, n); \
                     break;                                                  \
            case 3 : internal::cpy_front_##simd<true, true>(dstc, srcc, n);  \
                     break;                                                  \
            default :                                                        \
                     break;                                                  \
        }                                                                    \
                                                                             \
        std::size_t m = (n / (align * 2)) * (align * 2);                     \
        dstc += m;                                                           \
        srcc += m;                                                           \
        memmove_std(dstc, srcc, n % (align * 2));                            \
                                                                             \
        return dst;                                                          \
    }                                                                        \
                                                                             \
    char *dstc = static_cast<char *>(dst) + n;                               \
    const char *srcc = static_cast<const char *>(src) + n;                   \
    std::size_t offset = reinterpret_cast<uintptr_t>(dstc) % align;          \
    if (offset != 0) {                                                       \
        dstc -= offset;                                                      \
        srcc -= offset;                                                      \
        n -= offset;                                                         \
        memmove_std(dstc, srcc, offset);                                     \
    }                                                                        \
                                                                             \
    unsigned flag = CStringNonTemporalThreshold::instance().over(n);         \
    flag &= CStringNonTemporalThreshold::instance().over(dsta - srca);       \
    flag |= internal::cstring_is_aligned<align>(srcc);                       \
    switch (flag) {                                                          \
        case 0 : internal::cpy_back_##simd<false, false>(dstc, srcc, n);     \
                 break;                                                      \
        case 1 : internal::cpy_back_##simd<false, true >(dstc, srcc, n);     \
                 break;                                                      \
        case 2 : internal::cpy_back_##simd<true,  false>(dstc, srcc, n);     \
                 break;                                                      \
        case 3 : internal::cpy_back_##simd<true,  true >(dstc, srcc, n);     \
                 break;                                                      \
        default :                                                            \
                 break;                                                      \
    }                                                                        \
                                                                             \
    memmove_std(dst, src, n % (align * 2));                                  \
                                                                             \
    return dst;                                                              \
}

namespace vsmc {

namespace internal {

template <std::size_t Alignment>
inline unsigned cstring_is_aligned (const void *ptr)
{return reinterpret_cast<uintptr_t>(ptr) % Alignment == 0 ? 2 : 0;}

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

    CStringNonTemporalThreshold () : threshold_(0) {set();}

    CStringNonTemporalThreshold (const CStringNonTemporalThreshold &);

    CStringNonTemporalThreshold &operator= (
            const CStringNonTemporalThreshold &);
}; // class CStringNonTemporalThreshold

/// \brief Direct call to `std::memcpy`
/// \ingroup CString
inline void *memcpy_std (void *dst, const void *src, std::size_t n)
{
    if (dst == src || n == 0)
        return dst;

    return std::memcpy(dst, src, n);
}

/// \brief Direct call to `std::memmove`
/// \ingroup CString
inline void *memmove_std (void *dst, const void *src, std::size_t n)
{
    if (dst == src || n == 0)
        return dst;

    return std::memmove(dst, src, n);
}

#ifdef __SSE2__

namespace internal {

template <bool, bool>
inline void cpy_front_sse2 (void *, const void *, std::size_t);

template <bool, bool>
inline void cpy_back_sse2 (void *, const void *, std::size_t);

VSMC_DEFINE_UTILITY_CSTRING_SIMD(false, false, sse2, 16,
        double, __m128d, _mm_loadu_pd, _mm_store_pd)
VSMC_DEFINE_UTILITY_CSTRING_SIMD(false, true, sse2, 16,
        double, __m128d, _mm_loadu_pd, _mm_stream_pd)
VSMC_DEFINE_UTILITY_CSTRING_SIMD(true, false, sse2, 16,
        double, __m128d, _mm_load_pd, _mm_store_pd)
VSMC_DEFINE_UTILITY_CSTRING_SIMD(true, true, sse2, 16,
        double, __m128d, _mm_load_pd, _mm_stream_pd)

} // namespace vsmc::internal

/// \brief SSE2 optimized `memcpy` with non-temporal store for large buffers
/// \ingroup CString
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY(sse2, 16)

/// \brief SSE2 optimized `memmove` with non-temporal store for large buffers
/// \ingroup CString
VSMC_DEFINE_UTILITY_CSTRING_MEMMOVE(sse2, 16)

#endif // __SSE2__

#ifdef __SSE3__

namespace internal {

template <bool, bool>
inline void cpy_front_sse3 (void *, const void *, std::size_t);

template <bool, bool>
inline void cpy_back_sse3 (void *, const void *, std::size_t);

VSMC_DEFINE_UTILITY_CSTRING_SIMD(false, false, sse3, 16,
        __m128i, __m128i, _mm_lddqu_si128, _mm_store_si128)
VSMC_DEFINE_UTILITY_CSTRING_SIMD(false, true, sse3, 16,
        __m128i, __m128i, _mm_lddqu_si128, _mm_stream_si128)
VSMC_DEFINE_UTILITY_CSTRING_SIMD(true, false, sse3, 16,
        double, __m128d, _mm_load_pd, _mm_store_pd)
VSMC_DEFINE_UTILITY_CSTRING_SIMD(true, true, sse3, 16,
        double, __m128d, _mm_load_pd, _mm_stream_pd)

} // namespace vsmc::internal

/// \brief SSE3 optimized `memcpy` with non-temporal store for large buffers
/// \ingroup CString
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY(sse3, 16)

/// \brief SSE3 optimized `memmove` with non-temporal store for large buffers
/// \ingroup CString
VSMC_DEFINE_UTILITY_CSTRING_MEMMOVE(sse3, 16)

#endif __SSE3__

#ifdef __AVX__

namespace internal {

template <bool, bool>
inline void cpy_front_avx (void *, const void *, std::size_t);

template <bool, bool>
inline void cpy_back_avx (void *, const void *, std::size_t);

VSMC_DEFINE_UTILITY_CSTRING_SIMD(false, false, avx, 32,
        __m256i, __m256i, _mm256_lddqu_si256, _mm256_store_si256)
VSMC_DEFINE_UTILITY_CSTRING_SIMD(false, true, avx, 32,
        __m256i, __m256i, _mm256_lddqu_si256, _mm256_stream_si256)
VSMC_DEFINE_UTILITY_CSTRING_SIMD(true, false, avx, 32,
        double, __m256d, _mm256_load_pd, _mm256_store_pd)
VSMC_DEFINE_UTILITY_CSTRING_SIMD(true, true, avx, 32,
        double, __m256d, _mm256_load_pd, _mm256_stream_pd)

} // namespace vsmc::internal

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

    void *memcpy (void *dst, const void *src, std::size_t n) const
    {
        if (n < VSMC_CSTRING_STD_THRESHOLD)
            return std::memcpy(dst, src, n);

        return memcpy_(dst, src, n);
    }

    private :

    void *(*memcpy_) (void *, const void *, std::size_t);
    void *(*memmove_) (void *, const void *, std::size_t);

    CStringRuntimeDispatch () : memcpy_(memcpy_std)
    {
#ifdef __AVX__
        if (CPUID::has_feature<CPUIDFeatureAVX>()) {
            memcpy_ = ::vsmc::memcpy_avx;
            memmove_ = ::vsmc::memmove_avx;

            return;
        }
#endif // __AVX__

#ifdef __SSE2__
        if (CPUID::has_feature<CPUIDFeatureSSE2>()) {
            memcpy_ = ::vsmc::memcpy_sse2;
            memmove_ = ::vsmc::memmove_sse2;

            return;
        }
#endif // __SSE2__

#ifdef __SSE3__
        if (CPUID::has_feature<CPUIDFeatureSSE3>()) {
            memcpy_ = ::vsmc::memcpy_sse3;
            memmove_ = ::vsmc::memmove_sse3;

            return;
        }
#endif // __SSE2__

        memcpy_ = memcpy_std;
        memmove_ = memmove_std;
    }

    CStringRuntimeDispatch (const CStringRuntimeDispatch &);

    CStringRuntimeDispatch &operator= (const CStringRuntimeDispatch &);
}; // class CStringRuntimeDispatc

} // namespace vsmc::internal

/// \brief SIMD optimized `memcpy` with non-temporal store for large buffers
/// \ingroup CString
inline void *memcpy (void *dst, const void *src, std::size_t n)
{
#if VSMC_CSTRING_RUNTIME_DISPATCH
    return internal::CStringRuntimeDispatch::instance().memcpy(dst, src, n);
#else
    if (n < VSMC_CSTRING_STD_THRESHOLD)
        return memcpy_std(dst, src, n);
#if defined(__AVX__)
    return memcpy_avx(dst, src, n);
#elif defined(__SSE3__)
    return memcpy_sse3(dst, src, n);
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
#if VSMC_CSTRING_RUNTIME_DISPATCH
    return internal::CStringRuntimeDispatch::instance().memmove(dst, src, n);
#else
    if (n < VSMC_CSTRING_STD_THRESHOLD)
        return memmove_std(dst, src, n);
#if defined(__AVX__)
    return memmove_avx(dst, src, n);
#elif defined(__SSE3__)
    return memmove_sse3(dst, src, n);
#elif defined(__SSE2__)
    return memmove_sse2(dst, src, n);
#else
    return memmove_std(dst, src, n);
#endif // defined(__AVX__)
#endif // VSMC_CSTRING_RUNTIME_DISPATCH
}

} // namespace vsmc

#endif // VSMC_UTILITY_CSTRING_HPP
