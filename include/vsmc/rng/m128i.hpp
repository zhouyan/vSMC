#ifndef VSMC_RNG_M128I_HPP
#define VSMC_RNG_M128I_HPP

#include <vsmc/rng/common.hpp>
#include <wmmintrin.h>

#define VSMC_STATIC_ASSERT_RNG_M128I_PACK(T, N) \
    VSMC_STATIC_ASSERT((sizeof(T) * N >= sizeof(__m128i)),                   \
            TRY_TO_PACK_OR_UNPACK_A_TOO_SMALL_VECTOR_INTO_OR_FROM_m128i)

namespace vsmc {

namespace internal {

template <typename T, std::size_t N>
void pack (const StaticVector<T, N> &c, __m128i &m)
{
    VSMC_STATIC_ASSERT_RNG_M128I_PACK(T, N);
    _mm_storeu_si128(&m, *(reinterpret_cast<const __m128i *>(c.data())));
}

template <typename T, std::size_t N>
void unpack (const __m128i &m, StaticVector<T, N> &c)
{
    VSMC_STATIC_ASSERT_RNG_M128I_PACK(T, N);
    _mm_storeu_si128(reinterpret_cast<__m128i *>(c.data()), m);
}

} // namespace vsmc::internal

} // namespace vsmc

#endif // VSMC_RNG_M128I_HPP
