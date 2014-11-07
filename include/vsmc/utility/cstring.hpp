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
/// The implementaions are optimzied with either SSE or AVX. Three groups of
/// functions are provided.
///
/// - `memcpy_std` etc., they simply call `std::memcpy` etc.
/// - `memcpy_sse` etc., they are avialable if at least SSE2 is supported and
/// are optimized with SSE instructions
/// - `memcpy_avx` etc., they are avialable if at least AVX is supported and
///
/// are optimized with AVX instructions
/// There are also generic `vsmc::memcpy` etc. They dispatch the call based on
/// the following rules.
///
/// - If buffer size (in bytes) is smaller than `VSMC_CSTRING_STD_THRESHOLD`,
/// call `memcpy_std` etc.
/// - Else if AVX is available, then call `memcpy_avx` etc.
/// - Else if SSE2 is available, then call `memcpy_sse` etc.
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

#define VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_SSE(srca, nt, load, store) \
template <>                                                                  \
inline void *memcpy_sse<srca, nt> (                                          \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    double *dstd = static_cast<double *>(dst);                               \
    const double *srcd = static_cast<const double *>(src);                   \
                                                                             \
    std::size_t nm = n / 32;                                                 \
    for (std::size_t i = 0; i != nm; ++i, dstd += 4, srcd += 4) {            \
        __m128d m1 = _mm_##load##_pd(srcd);                                  \
        __m128d m2 = _mm_##load##_pd(srcd + 2);                              \
        _mm_##store##_pd(dstd, m1);                                          \
        _mm_##store##_pd(dstd + 2, m2);                                      \
    }                                                                        \
    memcpy_std(dstd, srcd, n % 32);                                          \
                                                                             \
    return dst;                                                              \
}

#endif // __SSE2__

#ifdef __AVX__
#include <immintrin.h>

#define VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_AVX(srca, nt, load, store) \
template <>                                                                  \
inline void *memcpy_avx<srca, nt> (                                          \
        void *dst, const void *src, std::size_t n)                           \
{                                                                            \
    double *dstd = static_cast<double *>(dst);                               \
    const double *srcd = static_cast<const double *>(src);                   \
                                                                             \
    std::size_t nm = n / 64;                                                 \
    for (std::size_t i = 0; i != nm; ++i, dstd += 8, srcd += 8) {            \
        __m256d m1 = _mm256_##load##_pd(srcd);                               \
        __m256d m2 = _mm256_##load##_pd(srcd + 4);                           \
        _mm256_##store##_pd(dstd, m1);                                       \
        _mm256_##store##_pd(dstd + 4, m2);                                   \
    }                                                                        \
    memcpy_std(dstd, srcd, n % 64);                                          \
    _mm256_zeroupper();                                                      \
                                                                             \
    return dst;                                                              \
}

#endif // __AVX__


namespace vsmc {

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

inline void *memcpy_std (void *dst, const void *src, std::size_t n)
{
    if (n == 0)
        return dst;

    return std::memcpy(dst, src, n);
}

#ifdef __SSE2__

namespace internal {

inline unsigned cstring_is_aligned_sse (const void *ptr)
{return reinterpret_cast<uintptr_t>(ptr) % 16 == 0 ? 1 : 0;}

template <bool, bool>
inline void *memcpy_sse (void *, const void *, std::size_t);

VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_SSE(false, false, loadu, store)
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_SSE(false, true,  loadu, stream)
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_SSE(true,  false, load,  store)
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_SSE(true,  true,  load,  stream)

} // namespace vsmc::internal

/// \brief SSE optimized `memcpy` with non-temporal store for large buffers
/// \ingroup CString
inline void *memcpy_sse (void *dst, const void *src, std::size_t n)
{
    if (n < 16)
        return memcpy_std(dst, src, n);

    void *dstd = dst;
    const void *srcd = src;
    std::size_t offset = reinterpret_cast<uintptr_t>(dst) % 16;
    if (offset != 0) {
        offset = 16 - offset;
        memcpy_std(dst, src, offset);
        n -= offset;
        dstd = static_cast<void *>(static_cast<char *>(dst) + offset);
        srcd = static_cast<const void *>(
                static_cast<const char *>(src) + offset);
    }

    unsigned flag_src = internal::cstring_is_aligned_sse(srcd);
    unsigned flag_nt = CStringNonTemporalThreshold::instance().over(n);
    unsigned flag = (flag_src << 1) | flag_nt;
    switch (flag) {
        case 0 : internal::memcpy_sse<false, false>(dstd, srcd, n); break;
        case 1 : internal::memcpy_sse<false, true >(dstd, srcd, n); break;
        case 2 : internal::memcpy_sse<true,  false>(dstd, srcd, n); break;
        case 3 : internal::memcpy_sse<true,  true >(dstd, srcd, n); break;
        default : break;
    }

    return dst;
}

#endif

#ifdef __AVX__

namespace internal {

inline unsigned cstring_is_aligned_avx (const void *ptr)
{return reinterpret_cast<uintptr_t>(ptr) % 32 == 0 ? 1 : 0;}

template <bool, bool>
inline void *memcpy_avx (void *, const void *, std::size_t);

VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_AVX(false, false, loadu, store)
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_AVX(false, true,  loadu, stream)
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_AVX(true,  false, load,  store)
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_AVX(true,  true,  load,  stream)

} // namespace vsmc::internal

/// \brief AVX optimized `memcpy` with non-temporal store for large buffers
/// \ingroup CString
inline void *memcpy_avx (void *dst, const void *src, std::size_t n)
{
    if (n < 32)
        return memcpy_std(dst, src, n);

    void *dstd = dst;
    const void *srcd = src;
    std::size_t offset = reinterpret_cast<uintptr_t>(dst) % 32;
    if (offset != 0) {
        offset = 32 - offset;
        memcpy_std(dst, src, offset);
        n -= offset;
        dstd = static_cast<void *>(static_cast<char *>(dst) + offset);
        srcd = static_cast<const void *>(
                static_cast<const char *>(src) + offset);
    }

    unsigned flag_src = internal::cstring_is_aligned_avx(srcd);
    unsigned flag_nt = CStringNonTemporalThreshold::instance().over(n);
    unsigned flag = (flag_src << 1) | flag_nt;
    switch (flag) {
        case 0 : internal::memcpy_avx<false, false>(dstd, srcd, n); break;
        case 1 : internal::memcpy_avx<false, true >(dstd, srcd, n); break;
        case 2 : internal::memcpy_avx<true,  false>(dstd, srcd, n); break;
        case 3 : internal::memcpy_avx<true,  true >(dstd, srcd, n); break;
        default : break;
    }

    return dst;
}

#endif

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

    CStringRuntimeDispatch () : memcpy_(memcpy_std)
    {
#ifdef __AVX__
        if (CPUID::has_feature<CPUIDFeatureAVX>()) {
            memcpy_ = ::vsmc::memcpy_avx;

            return;
        }
#endif

#ifdef __SSE2__
        if (CPUID::has_feature<CPUIDFeatureSSE2>()) {
            memcpy_ = ::vsmc::memcpy_sse;

            return;
        }
#endif

        memcpy_ = memcpy_std;
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
#elif defined(__SSE2__)
    return memcpy_sse(dst, src, n);
#else
    return memcpy_std(dst, src, n);
#endif
#endif
}

} // namespace vsmc

#endif // VSMC_UTILITY_CSTRING_HPP
