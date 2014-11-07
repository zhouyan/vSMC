//============================================================================
// include/vsmc/utility/cstring.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distributed under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_UTILITY_CSTRING_HPP
#define VSMC_UTILITY_CSTRING_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/cpuid.hpp>

/// \brief Shall functions in this module do runtime dispatch
/// \ingroup Config
#ifndef VSMC_CSTRING_RUNTIME_DISPACH
#define VSMC_CSTRING_RUNTIME_DISPACH 0
#endif

/// \brief Threshold of bytes below which to call functions in system libraries
/// \ingroup Config
#ifndef VSMC_CSTRING_SYSTEM_THRESHOLD
#define VSMC_CSTRING_SYSTEM_THRESHOLD (1 << 10)
#endif

#ifdef __SSE2__
#include <emmintrin.h>

#define VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_SSE2(srca, nt, load, store) \
template <>                                                                  \
inline void *memcpy_sse2<srca, nt> (                                         \
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
    internal::memcpy_generic(dstd, srcd, n % 32);                            \
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
    internal::memcpy_generic(dstd, srcd, n % 64);                            \
    _mm256_zeroupper();                                                      \
                                                                             \
    return dst;                                                              \
}

#endif // __AVX__

#define VSMC_DEFINE_UTILITY_CSTRING_MEMCPY(align, SIMD, simd) \
inline unsigned memcpy_is_aligned_##simd (const void *ptr)                   \
{return reinterpret_cast<uintptr_t>(ptr) % align == 0 ? 1 : 0;}              \
                                                                             \
template <bool, bool>                                                        \
inline void *memcpy_##simd (void *, const void *, std::size_t);              \
                                                                             \
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_##SIMD(false, false, loadu, store)        \
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_##SIMD(false, true,  loadu, stream)       \
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_##SIMD(true,  false, load,  store)        \
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_##SIMD(true,  true,  load,  stream)       \
                                                                             \
inline void *memcpy_##simd (void *dst, const void *src, std::size_t n)       \
{                                                                            \
    if (n < align)                                                           \
        return memcpy_generic(dst, src, n);                                  \
                                                                             \
    void *dstd = dst;                                                        \
    const void *srcd = src;                                                  \
    std::size_t offset = reinterpret_cast<uintptr_t>(dst) % align;           \
    if (offset != 0) {                                                       \
        offset = align - offset;                                             \
        memcpy_generic(dst, src, offset);                                    \
        n -= offset;                                                         \
        dstd = static_cast<void *>(static_cast<char *>(dst) + offset);       \
        srcd = static_cast<const void *>(                                    \
                static_cast<const char *>(src) + offset);                    \
    }                                                                        \
                                                                             \
    unsigned flag_src = internal::memcpy_is_aligned_##simd(srcd);            \
    unsigned flag_nt = CStringNonTemporalThreshold::instance().over(n);      \
    unsigned flag = (flag_src << 1) | flag_nt;                               \
                                                                             \
    switch (flag) {                                                          \
        case 0  : memcpy_##simd<false, false>(dstd, srcd, n); break;         \
        case 1  : memcpy_##simd<false, true >(dstd, srcd, n); break;         \
        case 2  : memcpy_##simd<true,  false>(dstd, srcd, n); break;         \
        case 3  : memcpy_##simd<true,  true >(dstd, srcd, n); break;         \
        default : break;                                                     \
    }                                                                        \
                                                                             \
    return dst;                                                              \
}

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

namespace internal {

inline void *memcpy_generic (void *dst, const void *src, std::size_t n)
{
    if (n == 0)
        return dst;

    return std::memcpy(dst, src, n);
}

#ifdef __SSE2__
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY(16, SSE2, sse2)
#endif

#ifdef __AVX__
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY(32, AVX, avx)
#endif

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
        if (n < VSMC_CSTRING_SYSTEM_THRESHOLD)
            return std::memcpy(dst, src, n);

        return memcpy_(dst, src, n);
    }

    private :

    void *(*memcpy_) (void *, const void *, std::size_t);

    CStringRuntimeDispatch () : memcpy_(memcpy_generic)
    {
#ifdef __AVX__
        if (CPUID::has_feature<CPUIDFeatureAVX>()) {
            memcpy_ = memcpy_avx;

            return;
        }
#endif

#ifdef __SSE2__
        if (CPUID::has_feature<CPUIDFeatureSSE2>()) {
            memcpy_ = memcpy_sse2;

            return;
        }
#endif

        memcpy_ = memcpy_generic;
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
    if (n < VSMC_CSTRING_SYSTEM_THRESHOLD)
        return internal::memcpy_generic(dst, src, n);
#if defined(__AVX__)
    return internal::memcpy_avx(dst, src, n);
#elif defined(__SSE2__)
    return internal::memcpy_sse2(dst, src, n);
#else
    return internal::memcpy_generic(dst, src, n);
#endif
#endif
}

} // namespace vsmc

#endif // VSMC_UTILITY_CSTRING_HPP
