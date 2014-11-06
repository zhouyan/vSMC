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

#ifdef __SSE2__
#include <emmintrin.h>

#define VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_SSE2(da, sa, nt, store, load) \
template <>                                                                  \
inline void *memcpy_sse2<da, sa, nt> (                                       \
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

#define VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_AVX(da, sa, nt, store, load) \
template <>                                                                  \
inline void *memcpy_avx<da, sa, nt> (                                        \
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
                                                                             \
    return dst;                                                              \
}

#endif // __AVX__

#define VSMC_DEFINE_UTILITY_CSTRING_MEMCPY(align, small, SIMD, simd) \
inline unsigned memcpy_is_aligned_##simd (const void *ptr)                   \
{return reinterpret_cast<uintptr_t>(ptr) % align == 0 ? 1 : 0;}              \
                                                                             \
template <bool, bool, bool>                                                  \
inline void *memcpy_##simd (void *, const void *, std::size_t);              \
                                                                             \
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_##SIMD(true,  true,  true,  stream, load) \
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_##SIMD(true,  false, true,  stream, loadu)\
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_##SIMD(true,  true,  false, store,  load) \
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_##SIMD(true,  false, false, store,  loadu)\
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_##SIMD(false, true,  false, storeu, load) \
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY_##SIMD(false, false, false, storeu, loadu)\
                                                                             \
inline void *memcpy_##simd (void *dst, const void *src, std::size_t n)       \
{                                                                            \
    if (n < small)                                                           \
        return internal::memcpy_generic(dst, src, n);                        \
                                                                             \
    unsigned flag_nt = MemcpyNonTemporalThreshold::instance().over(n);       \
    unsigned flag_dst = internal::memcpy_is_aligned_##simd(dst);             \
    unsigned flag_src = internal::memcpy_is_aligned_##simd(src);             \
    unsigned flag = (flag_dst << 2) | (flag_src << 1) | (flag_nt & flag_dst);\
                                                                             \
    switch (flag) {                                                          \
        case 7  : return memcpy_##simd<true,  true,  true >(dst, src, n);    \
        case 5  : return memcpy_##simd<true,  false, true >(dst, src, n);    \
        case 6  : return memcpy_##simd<true,  true,  false>(dst, src, n);    \
        case 4  : return memcpy_##simd<true,  false, false>(dst, src, n);    \
        case 2  : return memcpy_##simd<false, true,  false>(dst, src, n);    \
        default : return memcpy_##simd<false, false, false>(dst, src, n);    \
    }                                                                        \
}

namespace vsmc {

/// \brief The threshold of buffer size above which `memcpy` use non-temporal
/// instructions.
/// \ingroup CString
class MemcpyNonTemporalThreshold
{
    public :

    /// \brief Singleton instance
    ///
    /// \note If any `memcpy` functions in this module is to be called from
    /// multiple thread, then this member function need to be called at least
    /// once before entering threads.
    static MemcpyNonTemporalThreshold &instance ()
    {
        static MemcpyNonTemporalThreshold ntt;

        return ntt;
    }

    /// \brief Set the threshold to default
    ///
    /// \details
    /// By default, we set a pretty high threshold (the LLC size).
    void set ()
    {
        unsigned max_ecx = CPUID::max_cache_index();
        if (max_ecx >= 3)
            threshold_ = CPUID::cache_param(3).size();
        else if (max_ecx >= 2)
            threshold_ = CPUID::cache_param(2).size();
        else
            threshold_ = 1UL << 18; // 256K
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

    MemcpyNonTemporalThreshold () : threshold_(0) {set();}

    MemcpyNonTemporalThreshold (const MemcpyNonTemporalThreshold &);

    MemcpyNonTemporalThreshold &operator= (const MemcpyNonTemporalThreshold &);
}; // class MemcpyNonTemporalThreshold

namespace internal {

inline void *memcpy_char (void *dst, const void *src, std::size_t n)
{
    char *dstc = static_cast<char *>(dst);
    const char *srcc = static_cast<const char *>(src);
    for (std::size_t i = 0; i != n; ++i, ++dstc, ++srcc)
        *dstc = *srcc;

    return dst;
}

inline void *memcpy_generic (void *dst, const void *src, std::size_t n)
{
    double *dstd = static_cast<double *>(dst);
    const double *srcd = static_cast<const double *>(src);
    std::size_t nd = n / 8;
    for (std::size_t i = 0; i != nd; ++i, ++dstd, ++srcd)
        *dstd = *srcd;
    memcpy_char(dstd, srcd, n % 8);

    return dst;
}

#ifdef __SSE2__
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY(16, 32, SSE2, sse2)
#endif

#ifdef __AVX__
VSMC_DEFINE_UTILITY_CSTRING_MEMCPY(32, 64, AVX, avx)
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
    {return memcpy_(dst, src, n);}

    private :

    void *(*memcpy_) (void *, const void *, std::size_t);

    CStringRuntimeDispatch ()
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
