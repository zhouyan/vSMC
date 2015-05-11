//============================================================================
// vSMC/include/vsmc/rng/internal/m256i.hpp
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

#ifndef VSMC_RNG_INTERNAL_M256I_HPP
#define VSMC_RNG_INTERNAL_M256I_HPP

#include <vsmc/rng/internal/common.hpp>
#include <immintrin.h>

#define VSMC_STATIC_ASSERT_RNG_INTERNAL_M256I_PACK(Offset, T, N)              \
    VSMC_STATIC_ASSERT(                                                       \
        ((Offset < N) && (sizeof(T) * (N - Offset) >= sizeof(__m256i))),      \
        "TRY TO PACK OR UNPACK A TOO SMALL VECTOR INTO OR FROM m256i")

namespace vsmc
{

namespace internal
{

template <std::size_t Offset, typename T, std::size_t N>
inline void m256i_pack_a(const std::array<T, N> &c, __m256i &m)
{
    m = _mm256_load_si256(
        reinterpret_cast<const __m256i *>(c.data() + Offset));
}

template <std::size_t Offset, typename T, std::size_t N>
inline void m256i_pack_u(const std::array<T, N> &c, __m256i &m)
{
    m = _mm256_loadu_si256(
        reinterpret_cast<const __m256i *>(c.data() + Offset));
}

template <std::size_t Offset, typename T, std::size_t N>
inline void m256i_unpack_a(const __m256i &m, std::array<T, N> &c)
{
    _mm256_store_si256(reinterpret_cast<__m256i *>(c.data() + Offset), m);
}

template <std::size_t Offset, typename T, std::size_t N>
inline void m256i_unpack_u(const __m256i &m, std::array<T, N> &c)
{
    _mm256_storeu_si256(reinterpret_cast<__m256i *>(c.data() + Offset), m);
}

template <std::size_t Offset, typename T, std::size_t N>
inline void m256i_pack(const std::array<T, N> &c, __m256i &m)
{
    VSMC_STATIC_ASSERT_RNG_INTERNAL_M256I_PACK(Offset, T, N);
    m256i_pack_u<Offset>(c, m);
}

template <std::size_t Offset, typename T, std::size_t N>
inline void m256i_unpack(const __m256i &m, std::array<T, N> &c)
{
    VSMC_STATIC_ASSERT_RNG_INTERNAL_M256I_PACK(Offset, T, N);
    m256i_unpack_u<Offset>(m, c);
}

inline bool m256i_is_equal(const __m256i &a, const __m256i &b)
{
    std::array<std::uint64_t, 4> sa;
    std::array<std::uint64_t, 4> sb;
    m256i_unpack<0>(a, sa);
    m256i_unpack<0>(b, sb);

    return sa == sb;
}

template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &m256i_output(
    std::basic_ostream<CharT, Traits> &os, const __m256i &a)
{
    if (os.good()) {
        std::array<unsigned char, 32> sa;
        m256i_unpack<0>(a, sa);
        os << sa;
    }

    return os;
}

template <typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits> &m256i_input(
    std::basic_istream<CharT, Traits> &is, __m256i &a)
{
    if (is.good()) {
        std::array<unsigned char, 32> sa;
        is >> sa;
        if (is)
            m256i_pack<0>(sa, a);
    }

    return is;
}

template <typename IntType>
class M256I
{
    public:
    typedef IntType value_type;

    M256I() : value_(_mm256_setzero_si256()) {}

    template <typename T>
    M256I(T n,
        typename std::enable_if<std::is_integral<T>::value>::type * = nullptr)
        : value_(set1(n, std::integral_constant<std::size_t, sizeof(T)>()))
    {
    }

    template <typename T>
    M256I(T e3, T e2, T e1, T e0)
        : value_(_mm256_set_epi64x(static_cast<VSMC_INT64>(e3),
              static_cast<VSMC_INT64>(e2), static_cast<VSMC_INT64>(e1),
              static_cast<VSMC_INT64>(e0)))
    {
    }

    template <typename T>
    M256I(T e7, T e6, T e5, T e4, T e3, T e2, T e1, T e0)
        : value_(_mm256_set_epi32(static_cast<int>(e7), static_cast<int>(e6),
              static_cast<int>(e5), static_cast<int>(e4), static_cast<int>(e3),
              static_cast<int>(e2), static_cast<int>(e1),
              static_cast<int>(e0)))
    {
    }

    template <typename T>
    M256I(T e15, T e14, T e13, T e12, T e11, T e10, T e9, T e8, T e7, T e6,
        T e5, T e4, T e3, T e2, T e1, T e0)
        : value_(_mm256_set_epi16(static_cast<short>(e15),
              static_cast<short>(e14), static_cast<short>(e13),
              static_cast<short>(e12), static_cast<short>(e11),
              static_cast<short>(e10), static_cast<short>(e9),
              static_cast<short>(e8), static_cast<short>(e7),
              static_cast<short>(e6), static_cast<short>(e5),
              static_cast<short>(e4), static_cast<short>(e3),
              static_cast<short>(e2), static_cast<short>(e1),
              static_cast<short>(e0)))
    {
    }

    template <typename T>
    M256I(T e31, T e30, T e29, T e28, T e27, T e26, T e25, T e24, T e23, T e22,
        T e21, T e20, T e19, T e18, T e17, T e16, T e15, T e14, T e13, T e12,
        T e11, T e10, T e9, T e8, T e7, T e6, T e5, T e4, T e3, T e2, T e1,
        T e0)
        : value_(
              _mm256_set_epi8(static_cast<char>(e31), static_cast<char>(e30),
                  static_cast<char>(e29), static_cast<char>(e28),
                  static_cast<char>(e27), static_cast<char>(e26),
                  static_cast<char>(e25), static_cast<char>(e24),
                  static_cast<char>(e23), static_cast<char>(e22),
                  static_cast<char>(e21), static_cast<char>(e20),
                  static_cast<char>(e19), static_cast<char>(e18),
                  static_cast<char>(e17), static_cast<char>(e16),
                  static_cast<char>(e15), static_cast<char>(e14),
                  static_cast<char>(e13), static_cast<char>(e12),
                  static_cast<char>(e11), static_cast<char>(e10),
                  static_cast<char>(e9), static_cast<char>(e8),
                  static_cast<char>(e7), static_cast<char>(e6),
                  static_cast<char>(e5), static_cast<char>(e4),
                  static_cast<char>(e3), static_cast<char>(e2),
                  static_cast<char>(e1), static_cast<char>(e0)))
    {
    }

    M256I(const M256I<IntType> &) = default;
    M256I(const __m256i &value) : value_(value) {}

    M256I<IntType> &operator=(const M256I<IntType> &) = default;
    M256I<IntType> &operator=(const __m256i &value)
    {
        value_ = value;

        return *this;
    }

    M256I(M256I<IntType> &&) = default;
    M256I(__m256i &&value) : value_(std::move(value)) {}

    M256I<IntType> &operator=(M256I<IntType> &&) = default;
    M256I<IntType> &operator=(__m256i &&value)
    {
        value_ = std::move(value);

        return *this;
    }

    static constexpr std::size_t size()
    {
        return sizeof(__m256i) / sizeof(IntType);
    }

    __m256i &value() { return value_; }
    const __m256i &value() const { return value_; }

    private:
    __m256i value_;

    template <typename T>
    __m256i set1(T n, std::integral_constant<std::size_t, 1>)
    {
        return _mm256_set1_epi8(static_cast<char>(n));
    }

    template <typename T>
    __m256i set1(T n, std::integral_constant<std::size_t, 2>)
    {
        return _mm256_set1_epi16(static_cast<short>(n));
    }

    template <typename T>
    __m256i set1(T n, std::integral_constant<std::size_t, 4>)
    {
        return _mm256_set1_epi32(static_cast<int>(n));
    }

    template <typename T>
    __m256i set1(T n, std::integral_constant<std::size_t, 8>)
    {
        return _mm256_set1_epi64x(static_cast<long long>(n));
    }
}; // class M256I

template <typename T>
inline M256I<T> m256i_add(const M256I<T> &a, const M256I<T> &b,
    std::integral_constant<std::size_t, 1>)
{
    return M256I<T>(_mm256_add_epi8(a.value(), b.value()));
}

template <typename T>
inline M256I<T> m256i_add(const M256I<T> &a, const M256I<T> &b,
    std::integral_constant<std::size_t, 2>)
{
    return M256I<T>(_mm256_add_epi16(a.value(), b.value()));
}

template <typename T>
inline M256I<T> m256i_add(const M256I<T> &a, const M256I<T> &b,
    std::integral_constant<std::size_t, 4>)
{
    return M256I<T>(_mm256_add_epi32(a.value(), b.value()));
}

template <typename T>
inline M256I<T> m256i_add(const M256I<T> &a, const M256I<T> &b,
    std::integral_constant<std::size_t, 8>)
{
    return M256I<T>(_mm256_add_epi64(a.value(), b.value()));
}

template <typename T>
inline M256I<T> m256i_sub(const M256I<T> &a, const M256I<T> &b,
    std::integral_constant<std::size_t, 1>)
{
    return M256I<T>(_mm256_sub_epi8(a.value(), b.value()));
}

template <typename T>
inline M256I<T> m256i_sub(const M256I<T> &a, const M256I<T> &b,
    std::integral_constant<std::size_t, 2>)
{
    return M256I<T>(_mm256_sub_epi16(a.value(), b.value()));
}

template <typename T>
inline M256I<T> m256i_sub(const M256I<T> &a, const M256I<T> &b,
    std::integral_constant<std::size_t, 4>)
{
    return M256I<T>(_mm256_sub_epi32(a.value(), b.value()));
}

template <typename T>
inline M256I<T> m256i_sub(const M256I<T> &a, const M256I<T> &b,
    std::integral_constant<std::size_t, 8>)
{
    return M256I<T>(_mm256_sub_epi64(a.value(), b.value()));
}

template <typename T>
inline M256I<T> m256i_slli(
    const M256I<T> &a, int imm8, std::integral_constant<std::size_t, 1>)
{
    return M256I<T>(_mm256_slli_epi8(a.value(), imm8));
}

template <typename T>
inline M256I<T> m256i_slli(
    const M256I<T> &a, int imm8, std::integral_constant<std::size_t, 2>)
{
    return M256I<T>(_mm256_slli_epi16(a.value(), imm8));
}

template <typename T>
inline M256I<T> m256i_slli(
    const M256I<T> &a, int imm8, std::integral_constant<std::size_t, 4>)
{
    return M256I<T>(_mm256_slli_epi32(a.value(), imm8));
}

template <typename T>
inline M256I<T> m256i_slli(
    const M256I<T> &a, int imm8, std::integral_constant<std::size_t, 8>)
{
    return M256I<T>(_mm256_slli_epi64(a.value(), imm8));
}

template <typename T>
inline M256I<T> m256i_srli(
    const M256I<T> &a, int imm8, std::integral_constant<std::size_t, 1>)
{
    return M256I<T>(_mm256_srli_epi8(a.value(), imm8));
}

template <typename T>
inline M256I<T> m256i_srli(
    const M256I<T> &a, int imm8, std::integral_constant<std::size_t, 2>)
{
    return M256I<T>(_mm256_srli_epi16(a.value(), imm8));
}

template <typename T>
inline M256I<T> m256i_srli(
    const M256I<T> &a, int imm8, std::integral_constant<std::size_t, 4>)
{
    return M256I<T>(_mm256_srli_epi32(a.value(), imm8));
}

template <typename T>
inline M256I<T> m256i_srli(
    const M256I<T> &a, int imm8, std::integral_constant<std::size_t, 8>)
{
    return M256I<T>(_mm256_srli_epi64(a.value(), imm8));
}

template <typename T>
inline M256I<T> operator+(const M256I<T> &a, const M256I<T> &b)
{
    return m256i_add(a, b, std::integral_constant<std::size_t, sizeof(T)>());
}

template <typename T>
inline M256I<T> &operator+=(M256I<T> &a, const M256I<T> &b)
{
    a = a + b;

    return a;
}

template <typename T>
inline M256I<T> operator-(const M256I<T> &a, const M256I<T> &b)
{
    return m256i_sub(a, b, std::integral_constant<std::size_t, sizeof(T)>());
}

template <typename T>
inline M256I<T> &operator-=(M256I<T> &a, const M256I<T> &b)
{
    a = a - b;

    return a;
}

template <typename T>
inline M256I<T> operator&(const M256I<T> &a, const M256I<T> &b)
{
    return M256I<T>(_mm256_and_si256(a.value(), b.value()));
}

template <typename T>
inline M256I<T> &operator&=(M256I<T> &a, const M256I<T> &b)
{
    a = a & b;

    return a;
}

template <typename T>
inline M256I<T> operator|(const M256I<T> &a, const M256I<T> &b)
{
    return M256I<T>(_mm256_or_si256(a.value(), b.value()));
}

template <typename T>
inline M256I<T> &operator|=(M256I<T> &a, const M256I<T> &b)
{
    a = a | b;

    return a;
}

template <typename T>
inline M256I<T> operator^(const M256I<T> &a, const M256I<T> &b) {
    return M256I<T>(_mm256_xor_si256(a.value(), b.value()));
}

template <typename T>
inline M256I<T> &operator^=(M256I<T> &a, const M256I<T> &b)
{
    a = a ^ b;

    return a;
}

template <typename T>
inline M256I<T> operator<<(const M256I<T> &a, int imm8)
{
    return m256i_slli(
        a, imm8, std::integral_constant<std::size_t, sizeof(T)>());
}

template <typename T>
inline M256I<T> operator<<=(M256I<T> &a, int imm8)
{
    a = a << imm8;

    return a;
}

template <typename T>
inline M256I<T> operator>>(const M256I<T> &a, int imm8)
{
    return m256i_srli(
        a, imm8, std::integral_constant<std::size_t, sizeof(T)>());
}

template <typename T>
inline M256I<T> operator>>=(M256I<T> &a, int imm8)
{
    a = a << imm8;

    return a;
}

} // namespace vsmc::internal

} // namespace vsmc

#endif // VSMC_RNG_INTERNAL_M256I_HPP
