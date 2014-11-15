//============================================================================
// vSMC/include/vsmc/rng/m128i.hpp
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

#ifndef VSMC_RNG_M128I_HPP
#define VSMC_RNG_M128I_HPP

#include <vsmc/rng/internal/common.hpp>

#ifdef VSMC_MSVC
#include <intrin.h>
#else
#include <emmintrin.h>
#endif

#define VSMC_STATIC_ASSERT_RNG_M128I_PACK(Offset, T, N) \
    VSMC_STATIC_ASSERT(                                                      \
            ((Offset < N) && (sizeof(T) * (N - Offset) >= sizeof(__m128i))), \
            TRY_TO_PACK_OR_UNPACK_A_TOO_SMALL_VECTOR_INTO_OR_FROM_m128i)

namespace vsmc {

/// \brief Aligned pack
/// \ingroup RNG
template <std::size_t Offset, typename T, std::size_t N>
VSMC_STRONG_INLINE void m128i_pack_a (const Array<T, N> &c, __m128i &m)
{m = _mm_load_si128(reinterpret_cast<const __m128i *>(c.data() + Offset));}

/// \brief Unaligned pack
/// \ingroup RNG
template <std::size_t Offset, typename T, std::size_t N>
VSMC_STRONG_INLINE void m128i_pack_u (const Array<T, N> &c, __m128i &m)
{m = _mm_loadu_si128(reinterpret_cast<const __m128i *>(c.data() + Offset));}

/// \brief Aligned unpack
/// \ingroup RNG
template <std::size_t Offset, typename T, std::size_t N>
VSMC_STRONG_INLINE void m128i_unpack_a (const __m128i &m, Array<T, N> &c)
{_mm_store_si128(reinterpret_cast<__m128i *>(c.data() + Offset), m);}

/// \brief Unaligned unpack
/// \ingroup RNG
template <std::size_t Offset, typename T, std::size_t N>
VSMC_STRONG_INLINE void m128i_unpack_u (const __m128i &m, Array<T, N> &c)
{_mm_storeu_si128(reinterpret_cast<__m128i *>(c.data() + Offset), m);}

/// \brief Pack an Array into an __m128i object
/// \ingroup RNG
template <std::size_t Offset, typename T, std::size_t N>
VSMC_STRONG_INLINE void m128i_pack (const Array<T, N> &c, __m128i &m)
{
    VSMC_STATIC_ASSERT_RNG_M128I_PACK(Offset, T, N);
    m128i_pack_u<Offset>(c, m);
}

/// \brief Unpack an __m128i object into an Array
/// \ingroup RNG
template <std::size_t Offset, typename T, std::size_t N>
VSMC_STRONG_INLINE void m128i_unpack (const __m128i &m, Array<T, N> &c)
{
    VSMC_STATIC_ASSERT_RNG_M128I_PACK(Offset, T, N);
    m128i_unpack_u<Offset>(m, c);
}

/// \brief Compare two __m128i objects
/// \ingroup RNG
VSMC_STRONG_INLINE bool m128i_is_equal (const __m128i &a, const __m128i &b)
{
    Array<uint64_t, 2> sa;
    Array<uint64_t, 2> sb;
    m128i_unpack<0>(a, sa);
    m128i_unpack<0>(b, sb);

    return sa == sb;
}

/// \brief Write an __m128i object into an output stream as 16 bytes
/// unsigned integers
/// \ingroup RNG
template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &m128i_output (
        std::basic_ostream<CharT, Traits> &os, const __m128i &a)
{
    if (os.good()) {
        Array<unsigned char, 16> sa;
        m128i_unpack<0>(a, sa);
        os << sa;
    }

    return os;
}

/// \brief Input an __m128i object from an input stream as 16 bytes
/// unsigned integers written by m128i_output
/// \ingroup RNG
template <typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits> &m128i_input (
        std::basic_istream<CharT, Traits> &is, __m128i &a)
{
    if (is.good()) {
        Array<unsigned char, 16> sa;
        is >> sa;
        if (is) m128i_pack<0>(sa, a);
    }

    return is;
}

} // namespace vsmc

#endif // VSMC_RNG_M128I_HPP
