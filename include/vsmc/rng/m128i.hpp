#ifndef VSMC_RNG_M128I_HPP
#define VSMC_RNG_M128I_HPP

#include <vsmc/rng/common.hpp>
#include <wmmintrin.h>

#define VSMC_STATIC_ASSERT_RNG_M128I_PACK(T, N) \
    VSMC_STATIC_ASSERT((sizeof(T) * N >= sizeof(__m128i)),                   \
            TRY_TO_PACK_OR_UNPACK_A_TOO_SMALL_VECTOR_INTO_OR_FROM_m128i)

namespace vsmc {

namespace internal {

template <typename T, std::size_t N, typename Traits>
inline void pack (const StaticVector<T, N, Traits> &c, __m128i &m)
{
    VSMC_STATIC_ASSERT_RNG_M128I_PACK(T, N);
    m = _mm_loadu_si128(reinterpret_cast<const __m128i *>(c.data()));
}

template <typename T, std::size_t N, typename Traits>
inline void unpack (const __m128i &m, StaticVector<T, N, Traits> &c)
{
    VSMC_STATIC_ASSERT_RNG_M128I_PACK(T, N);
    _mm_storeu_si128(reinterpret_cast<__m128i *>(c.data()), m);
}

inline bool is_equal (const __m128i &a, const __m128i &b)
{
    StaticVector<uint64_t, 2> sa;
    StaticVector<uint64_t, 2> sb;
    unpack(a, sa);
    unpack(b, sb);

    return sa == sb;
}

template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &output_m128i (
        std::basic_ostream<CharT, Traits> &os, const __m128i &a)
{
    StaticVector<uint64_t, 2> sa;
    unpack(a, sa);
    if (os) os << sa;

    return os;
}

template <typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits> &input_m128i (
        std::basic_istream<CharT, Traits> &is, __m128i &a)
{
    StaticVector<uint64_t, 2> sa;
    if (is) is >> sa;
    if (is) pack(sa, a);

    return is;
}

} // namespace vsmc::internal

} // namespace vsmc

#endif // VSMC_RNG_M128I_HPP
