#ifndef VSMC_RNG_M128I_HPP
#define VSMC_RNG_M128I_HPP

#include <vsmc/rng/internal/common.hpp>
#include <emmintrin.h>

#define VSMC_STATIC_ASSERT_RNG_M128I_PACK(Offset, T, N) \
    VSMC_STATIC_ASSERT(                                                      \
            ((Offset < N) && (sizeof(T) * (N - Offset) >= sizeof(__m128i))), \
            TRY_TO_PACK_OR_UNPACK_A_TOO_SMALL_VECTOR_INTO_OR_FROM_m128i)

namespace vsmc {

namespace internal {

template <typename T>
struct is_aligned_16 :
    public cxx11::integral_constant<bool,
    cxx11::is_same<T, __m128i>::value ||
    cxx11::is_same<T, __m128d>::value ||
    cxx11::is_same<T, __m128>::value> {};

template <std::size_t Offset, typename T, std::size_t N, typename Traits>
inline void m128i_pack (const StaticVector<T, N, Traits> &c, __m128i &m,
        cxx11::true_type)
{m = _mm_load_si128(reinterpret_cast<const __m128i *>(c.data() + Offset));}

template <std::size_t Offset, typename T, std::size_t N, typename Traits>
inline void m128i_pack (const StaticVector<T, N, Traits> &c, __m128i &m,
        cxx11::false_type)
{
    const __m128i *src = reinterpret_cast<const __m128i *>(c.data() + Offset);
    if ((reinterpret_cast<uintptr_t>(src) & static_cast<uintptr_t>(0x0F)) == 0)
        m = _mm_load_si128(src);
    else
        m = _mm_loadu_si128(src);
}

template <std::size_t Offset, typename T, std::size_t N, typename Traits>
inline void m128i_unpack (const __m128i &m, StaticVector<T, N, Traits> &c,
        cxx11::true_type)
{_mm_store_si128(reinterpret_cast<__m128i *>(c.data() + Offset), m);}

template <std::size_t Offset, typename T, std::size_t N, typename Traits>
inline void m128i_unpack (const __m128i &m, StaticVector<T, N, Traits> &c,
        cxx11::false_type)
{
    __m128i *dst = reinterpret_cast<__m128i *>(c.data() + Offset);
    if ((reinterpret_cast<uintptr_t>(dst) & static_cast<uintptr_t>(0x0F)) == 0)
        _mm_store_si128(dst, m);
    else
        _mm_storeu_si128(dst, m);
}

} // namespace internal

/// \brief Pack a StaticVector into an __m128i object
/// \ingroup RNG
template <std::size_t Offset, typename T, std::size_t N, typename Traits>
inline void m128i_pack (const StaticVector<T, N, Traits> &c, __m128i &m)
{
    VSMC_STATIC_ASSERT_RNG_M128I_PACK(Offset, T, N);
    internal::m128i_pack<Offset>(c, m, internal::is_aligned_16<T>());
}

/// \brief Unpack an __m128i object into a StaticVector
/// \ingroup RNG
template <std::size_t Offset, typename T, std::size_t N, typename Traits>
inline void m128i_unpack (const __m128i &m, StaticVector<T, N, Traits> &c)
{
    VSMC_STATIC_ASSERT_RNG_M128I_PACK(Offset, T, N);
    internal::m128i_unpack<Offset>(m, c, internal::is_aligned_16<T>());
}

/// \brief Compare two __m128i objects
/// \ingroup RNG
inline bool m128i_is_equal (const __m128i &a, const __m128i &b)
{
    StaticVector<uint64_t, 2> sa;
    StaticVector<uint64_t, 2> sb;
    m128i_unpack<0>(a, sa);
    m128i_unpack<0>(b, sb);

    return sa == sb;
}

/// \brief Write an __m128i object into an output stream as two 64-bits
/// unsigned integers
/// \ingroup RNG
template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &m128i_output (
        std::basic_ostream<CharT, Traits> &os, const __m128i &a)
{
    if (os) {
        StaticVector<uint64_t, 2> sa;
        m128i_unpack<0>(a, sa);
        os << sa;
    }

    return os;
}

/// \brief Input an __m128i object from an input stream as two 64-bits
/// unsigned integers written by m128i_output
/// \ingroup RNG
template <typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits> &m128i_input (
        std::basic_istream<CharT, Traits> &is, __m128i &a)
{
    if (is) {
        StaticVector<uint64_t, 2> sa;
        is >> sa;
        if (is) m128i_pack<0>(sa, a);
    }

    return is;
}

} // namespace vsmc

#endif // VSMC_RNG_M128I_HPP
