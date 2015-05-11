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
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, value, OR PROFITS; OR BUSINESS
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

template <typename IntType>
class M128I
{
    public:
    typedef IntType value_type;

    M128I() : value_(_mm_setzero_si128()) {}

    template <typename T>
    M128I(T n,
        typename std::enable_if<std::is_integral<T>::value>::type * = nullptr)
        : value_(set1(n, std::integral_constant<std::size_t, sizeof(T)>()))
    {
    }

    template <typename T>
    M128I(T e1, T e0)
        : value_(_mm_set_epi64x(
              static_cast<VSMC_INT64>(e1), static_cast<VSMC_INT64>(e0)))
    {
    }

    template <typename T>
    M128I(T e3, T e2, T e1, T e0)
        : value_(_mm_set_epi32(static_cast<int>(e3), static_cast<int>(e2),
              static_cast<int>(e1), static_cast<int>(e0)))
    {
    }

    template <typename T>
    M128I(T e7, T e6, T e5, T e4, T e3, T e2, T e1, T e0)
        : value_(_mm_set_epi16(static_cast<short>(e7), static_cast<short>(e6),
              static_cast<short>(e5), static_cast<short>(e4),
              static_cast<short>(e3), static_cast<short>(e2),
              static_cast<short>(e1), static_cast<short>(e0)))
    {
    }

    template <typename T>
    M128I(T e15, T e14, T e13, T e12, T e11, T e10, T e9, T e8, T e7, T e6,
        T e5, T e4, T e3, T e2, T e1, T e0)
        : value_(_mm_set_epi8(static_cast<char>(e15), static_cast<char>(e14),
              static_cast<char>(e13), static_cast<char>(e12),
              static_cast<char>(e11), static_cast<char>(e10),
              static_cast<char>(e9), static_cast<char>(e8),
              static_cast<char>(e7), static_cast<char>(e6),
              static_cast<char>(e5), static_cast<char>(e4),
              static_cast<char>(e3), static_cast<char>(e2),
              static_cast<char>(e1), static_cast<char>(e0)))
    {
    }

    M128I(const M128I<IntType> &) = default;
    M128I(const __m128i &value) : value_(value) {}

    M128I<IntType> &operator=(const M128I<IntType> &) = default;
    M128I<IntType> &operator=(const __m128i &value)
    {
        value_ = value;

        return *this;
    }

    M128I(M128I<IntType> &&) = default;
    M128I(__m128i &&value) : value_(std::move(value)) {}

    M128I<IntType> &operator=(M128I<IntType> &&) = default;
    M128I<IntType> &operator=(__m128i &&value)
    {
        value_ = std::move(value);

        return *this;
    }

    static constexpr std::size_t size()
    {
        return sizeof(__m128i) / sizeof(IntType);
    }

    __m128i &value() { return value_; }
    const __m128i &value() const { return value_; }

    private:
    __m128i value_;

    template <typename T>
    __m128i set1(T n, std::integral_constant<std::size_t, 1>)
    {
        return _mm_set1_epi8(static_cast<char>(n));
    }

    template <typename T>
    __m128i set1(T n, std::integral_constant<std::size_t, 2>)
    {
        return _mm_set1_epi16(static_cast<short>(n));
    }

    template <typename T>
    __m128i set1(T n, std::integral_constant<std::size_t, 4>)
    {
        return _mm_set1_epi32(static_cast<int>(n));
    }

    template <typename T>
    __m128i set1(T n, std::integral_constant<std::size_t, 8>)
    {
        return _mm_set1_epi64x(static_cast<VSMC_INT64>(n));
    }
}; // class M128I

template <typename T>
inline M128I<T> m128i_add(const M128I<T> &a, const M128I<T> &b,
    std::integral_constant<std::size_t, 1>)
{
    return M128I<T>(_mm_add_epi8(a.value(), b.value()));
}

template <typename T>
inline M128I<T> m128i_add(const M128I<T> &a, const M128I<T> &b,
    std::integral_constant<std::size_t, 2>)
{
    return M128I<T>(_mm_add_epi16(a.value(), b.value()));
}

template <typename T>
inline M128I<T> m128i_add(const M128I<T> &a, const M128I<T> &b,
    std::integral_constant<std::size_t, 4>)
{
    return M128I<T>(_mm_add_epi32(a.value(), b.value()));
}

template <typename T>
inline M128I<T> m128i_add(const M128I<T> &a, const M128I<T> &b,
    std::integral_constant<std::size_t, 8>)
{
    return M128I<T>(_mm_add_epi64(a.value(), b.value()));
}

template <typename T>
inline M128I<T> m128i_sub(const M128I<T> &a, const M128I<T> &b,
    std::integral_constant<std::size_t, 1>)
{
    return M128I<T>(_mm_sub_epi8(a.value(), b.value()));
}

template <typename T>
inline M128I<T> m128i_sub(const M128I<T> &a, const M128I<T> &b,
    std::integral_constant<std::size_t, 2>)
{
    return M128I<T>(_mm_sub_epi16(a.value(), b.value()));
}

template <typename T>
inline M128I<T> m128i_sub(const M128I<T> &a, const M128I<T> &b,
    std::integral_constant<std::size_t, 4>)
{
    return M128I<T>(_mm_sub_epi32(a.value(), b.value()));
}

template <typename T>
inline M128I<T> m128i_sub(const M128I<T> &a, const M128I<T> &b,
    std::integral_constant<std::size_t, 8>)
{
    return M128I<T>(_mm_sub_epi64(a.value(), b.value()));
}

template <typename T>
inline M128I<T> m128i_slli(
    const M128I<T> &a, int imm8, std::integral_constant<std::size_t, 1>)
{
    return M128I<T>(_mm_slli_epi8(a.value(), imm8));
}

template <typename T>
inline M128I<T> m128i_slli(
    const M128I<T> &a, int imm8, std::integral_constant<std::size_t, 2>)
{
    return M128I<T>(_mm_slli_epi16(a.value(), imm8));
}

template <typename T>
inline M128I<T> m128i_slli(
    const M128I<T> &a, int imm8, std::integral_constant<std::size_t, 4>)
{
    return M128I<T>(_mm_slli_epi32(a.value(), imm8));
}

template <typename T>
inline M128I<T> m128i_slli(
    const M128I<T> &a, int imm8, std::integral_constant<std::size_t, 8>)
{
    return M128I<T>(_mm_slli_epi64(a.value(), imm8));
}

template <typename T>
inline M128I<T> m128i_srli(
    const M128I<T> &a, int imm8, std::integral_constant<std::size_t, 1>)
{
    return M128I<T>(_mm_srli_epi8(a.value(), imm8));
}

template <typename T>
inline M128I<T> m128i_srli(
    const M128I<T> &a, int imm8, std::integral_constant<std::size_t, 2>)
{
    return M128I<T>(_mm_srli_epi16(a.value(), imm8));
}

template <typename T>
inline M128I<T> m128i_srli(
    const M128I<T> &a, int imm8, std::integral_constant<std::size_t, 4>)
{
    return M128I<T>(_mm_srli_epi32(a.value(), imm8));
}

template <typename T>
inline M128I<T> m128i_srli(
    const M128I<T> &a, int imm8, std::integral_constant<std::size_t, 8>)
{
    return M128I<T>(_mm_srli_epi64(a.value(), imm8));
}

template <typename T>
inline M128I<T> operator+(const M128I<T> &a, const M128I<T> &b)
{
    return m128i_add(a, b, std::integral_constant<std::size_t, sizeof(T)>());
}

template <typename T>
inline M128I<T> &operator+=(M128I<T> &a, const M128I<T> &b)
{
    a = a + b;

    return a;
}

template <typename T>
inline M128I<T> operator-(const M128I<T> &a, const M128I<T> &b)
{
    return m128i_sub(a, b, std::integral_constant<std::size_t, sizeof(T)>());
}

template <typename T>
inline M128I<T> &operator-=(M128I<T> &a, const M128I<T> &b)
{
    a = a - b;

    return a;
}

template <typename T>
inline M128I<T> operator&(const M128I<T> &a, const M128I<T> &b)
{
    return M128I<T>(_mm_and_si128(a.value(), b.value()));
}

template <typename T>
inline M128I<T> &operator&=(M128I<T> &a, const M128I<T> &b)
{
    a = a & b;

    return a;
}

template <typename T>
inline M128I<T> operator|(const M128I<T> &a, const M128I<T> &b)
{
    return M128I<T>(_mm_or_si128(a.value(), b.value()));
}

template <typename T>
inline M128I<T> &operator|=(M128I<T> &a, const M128I<T> &b)
{
    a = a | b;

    return a;
}

template <typename T>
inline M128I<T> operator^(const M128I<T> &a, const M128I<T> &b) {
    return M128I<T>(_mm_xor_si128(a.value(), b.value()));
}

template <typename T>
inline M128I<T> &operator^=(M128I<T> &a, const M128I<T> &b)
{
    a = a ^ b;

    return a;
}

template <typename T>
inline M128I<T> operator<<(const M128I<T> &a, int imm8)
{
    return m128i_slli(
        a, imm8, std::integral_constant<std::size_t, sizeof(T)>());
}

template <typename T>
inline M128I<T> operator<<=(M128I<T> &a, int imm8)
{
    a = a << imm8;

    return a;
}

template <typename T>
inline M128I<T> operator>>(const M128I<T> &a, int imm8)
{
    return m128i_srli(
        a, imm8, std::integral_constant<std::size_t, sizeof(T)>());
}

template <typename T>
inline M128I<T> operator>>=(M128I<T> &a, int imm8)
{
    a = a << imm8;

    return a;
}

} // namespace vsmc::internal

} // namespace vsmc

#endif // VSMC_RNG_INTERNAL_M128I_HPP
