#ifndef VSMC_RNG_M128I_HPP
#define VSMC_RNG_M128I_HPP

#include <vsmc/rng/common.hpp>
#include <emmintrin.h>

#define VSMC_STATIC_ASSERT_RNG_M128I_PACK(Offset, T, N) \
    VSMC_STATIC_ASSERT(                                                      \
            ((Offset < N) && (sizeof(T) * (N - Offset) >= sizeof(__m128i))), \
            TRY_TO_PACK_OR_UNPACK_A_TOO_SMALL_VECTOR_INTO_OR_FROM_m128i)

namespace vsmc {

/// \brief Pack a StaticVector into an __m128i object
/// \ingroup RNG
template <std::size_t Offset, typename T, std::size_t N, typename Traits>
inline void m128i_pack (const StaticVector<T, N, Traits> &c, __m128i &m)
{
    VSMC_STATIC_ASSERT_RNG_M128I_PACK(Offset, T, N);
    m = _mm_loadu_si128(reinterpret_cast<const __m128i *>(c.data() + Offset));
}

/// \brief Unpack an __m128i object into a StaticVector
/// \ingroup RNG
template <std::size_t Offset, typename T, std::size_t N, typename Traits>
inline void m128i_unpack (const __m128i &m, StaticVector<T, N, Traits> &c)
{
    VSMC_STATIC_ASSERT_RNG_M128I_PACK(Offset, T, N);
    _mm_storeu_si128(reinterpret_cast<__m128i *>(c.data() + Offset), m);
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
    StaticVector<uint64_t, 2> sa;
    m128i_unpack<0>(a, sa);
    if (os) os << sa;

    return os;
}

/// \brief Input an __m128i object from an input stream as two 64-bits
/// unsigned integers written by m128i_output
/// \ingroup RNG
template <typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits> &m128i_input (
        std::basic_istream<CharT, Traits> &is, __m128i &a)
{
    StaticVector<uint64_t, 2> sa;
    if (is) is >> sa;
    if (is) m128i_pack<0>(sa, a);

    return is;
}

} // namespace vsmc

#endif // VSMC_RNG_M128I_HPP
