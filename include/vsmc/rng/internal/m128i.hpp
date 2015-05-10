//============================================================================
// vSMC/include/vsmc/rng/internal/m128i.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
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

#ifndef VSMC_RNG_INTERNAL_M128I_HPP
#define VSMC_RNG_INTERNAL_M128I_HPP

#include <vsmc/rng/internal/common.hpp>
#include <emmintrin.h>

#define VSMC_STATIC_ASSERT_RNG_INTERNAL_M128I_PACK(Offset, T, N)              \
    VSMC_STATIC_ASSERT(                                                       \
        ((Offset < N) && (sizeof(T) * (N - Offset) >= sizeof(__m128i))),      \
        "TRY TO PACK OR UNPACK A TOO SMALL VECTOR INTO OR FROM m128i")

namespace vsmc
{

namespace internal
{

template <std::size_t Offset, typename T, std::size_t N>
inline void m128i_pack_a(const std::array<T, N> &c, __m128i &m)
{
    m = _mm_load_si128(reinterpret_cast<const __m128i *>(c.data() + Offset));
}

template <std::size_t Offset, typename T, std::size_t N>
inline void m128i_pack_u(const std::array<T, N> &c, __m128i &m)
{
    m = _mm_loadu_si128(reinterpret_cast<const __m128i *>(c.data() + Offset));
}

template <std::size_t Offset, typename T, std::size_t N>
inline void m128i_unpack_a(const __m128i &m, std::array<T, N> &c)
{
    _mm_store_si128(reinterpret_cast<__m128i *>(c.data() + Offset), m);
}

template <std::size_t Offset, typename T, std::size_t N>
inline void m128i_unpack_u(const __m128i &m, std::array<T, N> &c)
{
    _mm_storeu_si128(reinterpret_cast<__m128i *>(c.data() + Offset), m);
}

template <std::size_t Offset, typename T, std::size_t N>
inline void m128i_pack(const std::array<T, N> &c, __m128i &m)
{
    VSMC_STATIC_ASSERT_RNG_INTERNAL_M128I_PACK(Offset, T, N);
    m128i_pack_u<Offset>(c, m);
}

template <std::size_t Offset, typename T, std::size_t N>
inline void m128i_unpack(const __m128i &m, std::array<T, N> &c)
{
    VSMC_STATIC_ASSERT_RNG_INTERNAL_M128I_PACK(Offset, T, N);
    m128i_unpack_u<Offset>(m, c);
}

inline bool m128i_is_equal(const __m128i &a, const __m128i &b)
{
    std::array<std::uint64_t, 2> sa;
    std::array<std::uint64_t, 2> sb;
    m128i_unpack<0>(a, sa);
    m128i_unpack<0>(b, sb);

    return sa == sb;
}

template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &m128i_output(
    std::basic_ostream<CharT, Traits> &os, const __m128i &a)
{
    if (os.good()) {
        std::array<unsigned char, 16> sa;
        m128i_unpack<0>(a, sa);
        os << sa;
    }

    return os;
}

template <typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits> &m128i_input(
    std::basic_istream<CharT, Traits> &is, __m128i &a)
{
    if (is.good()) {
        std::array<unsigned char, 16> sa;
        is >> sa;
        if (is)
            m128i_pack<0>(sa, a);
    }

    return is;
}

} // namespace vsmc::internal

} // namespace vsmc

#endif // VSMC_RNG_INTERNAL_M128I_HPP
