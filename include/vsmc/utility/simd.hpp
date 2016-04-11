//============================================================================
// vSMC/include/vsmc/utility/simd.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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

#ifndef VSMC_UTILITY_SIMD_HPP
#define VSMC_UTILITY_SIMD_HPP

#include <vsmc/internal/common.hpp>

#define VSMC_DEFINE_UTILITY_SIMD_INTEGER_BINARY_OP(                           \
    Type, CType, op, bin, assign)                                             \
    template <typename T>                                                     \
    inline Type &assign(Type &a, const Type &b)                               \
    {                                                                         \
        a = a op b;                                                           \
                                                                              \
        return a;                                                             \
    }                                                                         \
                                                                              \
    template <typename T>                                                     \
    inline Type bin(const Type &a, CType b)                                   \
    {                                                                         \
        Type x;                                                               \
        x.set1(b);                                                            \
                                                                              \
        return a + x;                                                         \
    }                                                                         \
                                                                              \
    template <typename T>                                                     \
    inline Type bin(CType a, const Type &b)                                   \
    {                                                                         \
        Type x;                                                               \
        x.set1(a);                                                            \
                                                                              \
        return x + b;                                                         \
    }                                                                         \
                                                                              \
    template <typename T>                                                     \
    inline Type &assign(Type &a, CType b)                                     \
    {                                                                         \
        a = a + b;                                                            \
                                                                              \
        return a;                                                             \
    }

#define VSMC_DEFINE_UTILITY_SIMD_REAL_BINARY_OP(Type, CType, op, bin, assign) \
    inline Type &assign(Type &a, const Type &b)                               \
    {                                                                         \
        a = a op b;                                                           \
                                                                              \
        return a;                                                             \
    }                                                                         \
                                                                              \
    inline Type bin(const Type &a, CType b)                                   \
    {                                                                         \
        Type x;                                                               \
        x.set1(b);                                                            \
                                                                              \
        return a + x;                                                         \
    }                                                                         \
                                                                              \
    inline Type bin(CType a, const Type &b)                                   \
    {                                                                         \
        Type x;                                                               \
        x.set1(a);                                                            \
                                                                              \
        return x + b;                                                         \
    }                                                                         \
                                                                              \
    inline Type &assign(Type &a, CType b)                                     \
    {                                                                         \
        a = a + b;                                                            \
                                                                              \
        return a;                                                             \
    }

#if VSMC_HAS_SSE2
#include <emmintrin.h>

namespace vsmc
{

/// \brief Using `__m128i` as integer vector
/// \ingroup SIMD
template <typename IntType = __m128i>
class M128I
{
    public:
    using value_type = IntType;

    M128I() = default;

    M128I(const __m128i &value) : value_(value) {}

    template <typename T>
    M128I(const M128I<T> &other) : value_(other.value())
    {
    }

    template <typename T>
    M128I<IntType> &operator=(const M128I<T> &other)
    {
        value_ = other.value();

        return *this;
    }

    static constexpr std::size_t size()
    {
        return sizeof(__m128i) / sizeof(IntType);
    }

    __m128i &value() { return value_; }
    const __m128i &value() const { return value_; }

    __m128i *data() { return &value_; }
    const __m128i *data() const { return &value_; }

    template <typename T>
    void load_a(const T *mem)
    {
        value_ = _mm_load_si128(reinterpret_cast<const __m128i *>(mem));
    }

    template <typename T>
    void load_u(const T *mem)
    {
        value_ = _mm_loadu_si128(reinterpret_cast<const __m128i *>(mem));
    }

    template <typename T>
    void load(const T *mem)
    {
        reinterpret_cast<std::uintptr_t>(mem) % 16 == 0 ? load_a(mem) :
                                                          load_u(mem);
    }

    template <typename T>
    void store_a(T *mem) const
    {
        _mm_store_si128(reinterpret_cast<__m128i *>(mem), value_);
    }

    template <typename T>
    void store_u(T *mem) const
    {
        _mm_storeu_si128(reinterpret_cast<__m128i *>(mem), value_);
    }

    template <typename T>
    void store(T *mem) const
    {
        reinterpret_cast<std::uintptr_t>(mem) % 16 == 0 ? store_a(mem) :
                                                          store_u(mem);
    }

    void set0() { value_ = _mm_setzero_si128(); }

    template <typename T>
    void set1(T n)
    {
        value_ = set1(
            n, std::integral_constant<int, std::numeric_limits<T>::digits>());
    }

    template <typename T>
    void set(T e1, T e0)
    {
        value_ = _mm_set_epi64x(
            static_cast<VSMC_INT64>(e1), static_cast<VSMC_INT64>(e0));
    }

    template <typename T>
    void set(T e3, T e2, T e1, T e0)
    {
        value_ = _mm_set_epi32(static_cast<int>(e3), static_cast<int>(e2),
            static_cast<int>(e1), static_cast<int>(e0));
    }

    template <typename T>
    void set(T e7, T e6, T e5, T e4, T e3, T e2, T e1, T e0)
    {
        value_ = _mm_set_epi16(static_cast<short>(e7), static_cast<short>(e6),
            static_cast<short>(e5), static_cast<short>(e4),
            static_cast<short>(e3), static_cast<short>(e2),
            static_cast<short>(e1), static_cast<short>(e0));
    }

    template <typename T>
    void set(T e15, T e14, T e13, T e12, T e11, T e10, T e9, T e8, T e7, T e6,
        T e5, T e4, T e3, T e2, T e1, T e0)
    {
        value_ = _mm_set_epi8(static_cast<char>(e15), static_cast<char>(e14),
            static_cast<char>(e13), static_cast<char>(e12),
            static_cast<char>(e11), static_cast<char>(e10),
            static_cast<char>(e9), static_cast<char>(e8),
            static_cast<char>(e7), static_cast<char>(e6),
            static_cast<char>(e5), static_cast<char>(e4),
            static_cast<char>(e3), static_cast<char>(e2),
            static_cast<char>(e1), static_cast<char>(e0));
    }

    private:
    __m128i value_;

    template <typename T>
    __m128i set1(T n, std::integral_constant<int, 8>)
    {
        return _mm_set1_epi8(static_cast<char>(n));
    }

    template <typename T>
    __m128i set1(T n, std::integral_constant<int, 16>)
    {
        return _mm_set1_epi16(static_cast<short>(n));
    }

    template <typename T>
    __m128i set1(T n, std::integral_constant<int, 32>)
    {
        return _mm_set1_epi32(static_cast<int>(n));
    }

    template <typename T>
    __m128i set1(T n, std::integral_constant<int, 64>)
    {
        return _mm_set1_epi64x(static_cast<VSMC_INT64>(n));
    }
}; // class M128I

namespace internal
{

template <typename T>
inline M128I<T> m128i_add(
    const M128I<T> &a, const M128I<T> &b, std::integral_constant<int, 8>)
{
    return M128I<T>(_mm_add_epi8(a.value(), b.value()));
}

template <typename T>
inline M128I<T> m128i_add(
    const M128I<T> &a, const M128I<T> &b, std::integral_constant<int, 16>)
{
    return M128I<T>(_mm_add_epi16(a.value(), b.value()));
}

template <typename T>
inline M128I<T> m128i_add(
    const M128I<T> &a, const M128I<T> &b, std::integral_constant<int, 32>)
{
    return M128I<T>(_mm_add_epi32(a.value(), b.value()));
}

template <typename T>
inline M128I<T> m128i_add(
    const M128I<T> &a, const M128I<T> &b, std::integral_constant<int, 64>)
{
    return M128I<T>(_mm_add_epi64(a.value(), b.value()));
}

template <typename T>
inline M128I<T> m128i_sub(
    const M128I<T> &a, const M128I<T> &b, std::integral_constant<int, 8>)
{
    return M128I<T>(_mm_sub_epi8(a.value(), b.value()));
}

template <typename T>
inline M128I<T> m128i_sub(
    const M128I<T> &a, const M128I<T> &b, std::integral_constant<int, 16>)
{
    return M128I<T>(_mm_sub_epi16(a.value(), b.value()));
}

template <typename T>
inline M128I<T> m128i_sub(
    const M128I<T> &a, const M128I<T> &b, std::integral_constant<int, 32>)
{
    return M128I<T>(_mm_sub_epi32(a.value(), b.value()));
}

template <typename T>
inline M128I<T> m128i_sub(
    const M128I<T> &a, const M128I<T> &b, std::integral_constant<int, 64>)
{
    return M128I<T>(_mm_sub_epi64(a.value(), b.value()));
}

template <typename T>
inline M128I<T> m128i_slli(
    const M128I<T> &a, int imm8, std::integral_constant<int, 8>)
{
    return M128I<T>(_mm_slli_epi8(a.value(), imm8));
}

template <typename T>
inline M128I<T> m128i_slli(
    const M128I<T> &a, int imm8, std::integral_constant<int, 16>)
{
    return M128I<T>(_mm_slli_epi16(a.value(), imm8));
}

template <typename T>
inline M128I<T> m128i_slli(
    const M128I<T> &a, int imm8, std::integral_constant<int, 32>)
{
    return M128I<T>(_mm_slli_epi32(a.value(), imm8));
}

template <typename T>
inline M128I<T> m128i_slli(
    const M128I<T> &a, int imm8, std::integral_constant<int, 64>)
{
    return M128I<T>(_mm_slli_epi64(a.value(), imm8));
}

template <typename T>
inline M128I<T> m128i_srli(
    const M128I<T> &a, int imm8, std::integral_constant<int, 8>)
{
    return M128I<T>(_mm_srli_epi8(a.value(), imm8));
}

template <typename T>
inline M128I<T> m128i_srli(
    const M128I<T> &a, int imm8, std::integral_constant<int, 16>)
{
    return M128I<T>(_mm_srli_epi16(a.value(), imm8));
}

template <typename T>
inline M128I<T> m128i_srli(
    const M128I<T> &a, int imm8, std::integral_constant<int, 32>)
{
    return M128I<T>(_mm_srli_epi32(a.value(), imm8));
}

template <typename T>
inline M128I<T> m128i_srli(
    const M128I<T> &a, int imm8, std::integral_constant<int, 64>)
{
    return M128I<T>(_mm_srli_epi64(a.value(), imm8));
}

} // namespace internal

template <typename T>
inline bool operator==(const M128I<T> &a, const M128I<T> &b)
{
    std::array<std::uint64_t, 2> sa;
    std::array<std::uint64_t, 2> sb;
    a.store_u(sa.data());
    b.store_u(sb.data());

    return sa == sb;
}

template <typename T>
inline bool operator!=(const M128I<T> &a, const M128I<T> &b)
{
    return !(a == b);
}

template <typename CharT, typename Traits, typename T>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const M128I<T> &a)
{
    if (!os)
        return os;

    std::array<T, M128I<T>::size()> sa;
    a.store_u(sa.data());
    os << sa;

    return os;
}

template <typename CharT, typename Traits, typename T>
inline std::basic_istream<CharT, Traits> &operator>>(
    std::basic_istream<CharT, Traits> &is, M128I<T> &a)
{
    if (!is)
        return is;

    std::array<T, M128I<T>::size()> sa;
    is >> sa;

    if (static_cast<bool>(is))
        a.load_u(sa.data());

    return is;
}

template <typename T>
inline M128I<T> operator+(const M128I<T> &a, const M128I<T> &b)
{
    return internal::m128i_add(
        a, b, std::integral_constant<int, std::numeric_limits<T>::digits>());
}

template <typename T>
inline M128I<T> operator-(const M128I<T> &a, const M128I<T> &b)
{
    return internal::m128i_sub(
        a, b, std::integral_constant<int, std::numeric_limits<T>::digits>());
}

template <typename T>
inline M128I<T> operator&(const M128I<T> &a, const M128I<T> &b)
{
    return M128I<T>(_mm_and_si128(a.value(), b.value()));
}

template <typename T>
inline M128I<T> operator|(const M128I<T> &a, const M128I<T> &b)
{
    return M128I<T>(_mm_or_si128(a.value(), b.value()));
}

template <typename T>
inline M128I<T> operator^(const M128I<T> &a, const M128I<T> &b)
{
    return M128I<T>(_mm_xor_si128(a.value(), b.value()));
}

template <typename T>
inline M128I<T> operator<<(const M128I<T> &a, int imm8)
{
    return internal::m128i_slli(a, imm8,
        std::integral_constant<int, std::numeric_limits<T>::digits>());
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
    return internal::m128i_srli(a, imm8,
        std::integral_constant<int, std::numeric_limits<T>::digits>());
}

template <typename T>
inline M128I<T> operator>>=(M128I<T> &a, int imm8)
{
    a = a << imm8;

    return a;
}

VSMC_DEFINE_UTILITY_SIMD_INTEGER_BINARY_OP(
    M128I<T>, T, +, operator+, operator+=)
VSMC_DEFINE_UTILITY_SIMD_INTEGER_BINARY_OP(
    M128I<T>, T, -, operator-, operator-=)
VSMC_DEFINE_UTILITY_SIMD_INTEGER_BINARY_OP(
    M128I<T>, T, &, operator&, operator&=)
VSMC_DEFINE_UTILITY_SIMD_INTEGER_BINARY_OP(
    M128I<T>, T, |, operator|, operator|=)
VSMC_DEFINE_UTILITY_SIMD_INTEGER_BINARY_OP(
    M128I<T>, T, ^, operator^, operator^=)

/// \brief `__m128`
/// \ingroup SIMD
class M128
{
    public:
    M128() = default;

    M128(const __m128 &value) : value_(value) {}

    static constexpr std::size_t size() { return 4; }

    __m128 &value() { return value_; }
    const __m128 &value() const { return value_; }

    __m128 *data() { return &value_; }
    const __m128 *data() const { return &value_; }

    template <typename T>
    void load_a(const T *mem)
    {
        value_ = _mm_load_ps(reinterpret_cast<const float *>(mem));
    }

    template <typename T>
    void load_u(const T *mem)
    {
        value_ = _mm_loadu_ps(reinterpret_cast<const float *>(mem));
    }

    template <typename T>
    void load(const T *mem)
    {
        reinterpret_cast<std::uintptr_t>(mem) % 16 == 0 ? load_a(mem) :
                                                          load_u(mem);
    }

    template <typename T>
    void store_a(T *mem) const
    {
        _mm_store_ps(reinterpret_cast<float *>(mem), value_);
    }

    template <typename T>
    void store_u(T *mem) const
    {
        _mm_storeu_ps(reinterpret_cast<float *>(mem), value_);
    }

    template <typename T>
    void store(T *mem) const
    {
        reinterpret_cast<std::uintptr_t>(mem) % 16 == 0 ? store_a(mem) :
                                                          store_u(mem);
    }

    void set0() { value_ = _mm_setzero_ps(); }

    void set1(float e) { value_ = _mm_set1_ps(e); }

    void set(float e3, float e2, float e1, float e0)
    {
        value_ = _mm_set_ps(e3, e2, e1, e0);
    }

    private:
    __m128 value_;
}; // class M128

inline bool operator==(const M128 &a, const M128 &b)
{
    std::array<float, 4> sa;
    std::array<float, 4> sb;
    a.store_u(sa.data());
    b.store_u(sb.data());

    return sa == sb;
}

inline bool operator!=(const M128 &a, const M128 &b) { return !(a == b); }

template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const M128 &a)
{
    if (!os)
        return os;

    std::array<float, 4> sa;
    a.store_u(sa.data());
    os << sa;

    return os;
}

template <typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits> &operator>>(
    std::basic_istream<CharT, Traits> &is, M128 &a)
{
    if (!is)
        return is;

    std::array<float, 4> sa;
    is >> sa;

    if (static_cast<bool>(is))
        a.load_u(sa.data());

    return is;
}

inline M128 operator+(const M128 &a, const M128 &b)
{
    return M128(_mm_add_ps(a.value(), b.value()));
}

inline M128 operator-(const M128 &a, const M128 &b)
{
    return M128(_mm_sub_ps(a.value(), b.value()));
}

inline M128 operator*(const M128 &a, const M128 &b)
{
    return M128(_mm_mul_ps(a.value(), b.value()));
}

inline M128 operator/(const M128 &a, const M128 &b)
{
    return M128(_mm_div_ps(a.value(), b.value()));
}

VSMC_DEFINE_UTILITY_SIMD_REAL_BINARY_OP(M128, float, +, operator+, operator+=)
VSMC_DEFINE_UTILITY_SIMD_REAL_BINARY_OP(M128, float, -, operator-, operator-=)
VSMC_DEFINE_UTILITY_SIMD_REAL_BINARY_OP(M128, float, *, operator*, operator*=)
VSMC_DEFINE_UTILITY_SIMD_REAL_BINARY_OP(M128, float, /, operator/, operator/=)

/// \brief `__m128d`
/// \ingroup SIMD
class M128D
{
    public:
    M128D() = default;

    M128D(const __m128d &value) : value_(value) {}

    static constexpr std::size_t size() { return 2; }

    __m128d &value() { return value_; }
    const __m128d &value() const { return value_; }

    __m128d *data() { return &value_; }
    const __m128d *data() const { return &value_; }

    template <typename T>
    void load_a(const T *mem)
    {
        value_ = _mm_load_pd(reinterpret_cast<const double *>(mem));
    }

    template <typename T>
    void load_u(const T *mem)
    {
        value_ = _mm_loadu_pd(reinterpret_cast<const double *>(mem));
    }

    template <typename T>
    void load(const T *mem)
    {
        reinterpret_cast<std::uintptr_t>(mem) % 16 == 0 ? load_a(mem) :
                                                          load_u(mem);
    }

    template <typename T>
    void store_a(T *mem) const
    {
        _mm_store_pd(reinterpret_cast<double *>(mem), value_);
    }

    template <typename T>
    void store_u(T *mem) const
    {
        _mm_storeu_pd(reinterpret_cast<double *>(mem), value_);
    }

    template <typename T>
    void store(T *mem) const
    {
        reinterpret_cast<std::uintptr_t>(mem) % 16 == 0 ? store_a(mem) :
                                                          store_u(mem);
    }

    void set0() { value_ = _mm_setzero_pd(); }

    void set1(double e) { value_ = _mm_set1_pd(e); }

    void set(double e1, double e0) { value_ = _mm_set_pd(e1, e0); }

    private:
    __m128d value_;
}; // class M128D

inline bool operator==(const M128D &a, const M128D &b)
{
    std::array<double, 2> sa;
    std::array<double, 2> sb;
    a.store_u(sa.data());
    b.store_u(sb.data());

    return sa == sb;
}

inline bool operator!=(const M128D &a, const M128D &b) { return !(a == b); }

template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const M128D &a)
{
    if (!os)
        return os;

    std::array<double, 2> sa;
    a.store_u(sa.data());
    os << sa;

    return os;
}

template <typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits> &operator>>(
    std::basic_istream<CharT, Traits> &is, M128D &a)
{
    if (!is)
        return is;

    std::array<double, 2> sa;
    is >> sa;

    if (static_cast<bool>(is))
        a.load_u(sa.data());

    return is;
}

inline M128D operator+(const M128D &a, const M128D &b)
{
    return M128D(_mm_add_pd(a.value(), b.value()));
}

inline M128D operator-(const M128D &a, const M128D &b)
{
    return M128D(_mm_sub_pd(a.value(), b.value()));
}

inline M128D operator*(const M128D &a, const M128D &b)
{
    return M128D(_mm_mul_pd(a.value(), b.value()));
}

inline M128D operator/(const M128D &a, const M128D &b)
{
    return M128D(_mm_div_pd(a.value(), b.value()));
}

VSMC_DEFINE_UTILITY_SIMD_REAL_BINARY_OP(
    M128D, double, +, operator+, operator+=)
VSMC_DEFINE_UTILITY_SIMD_REAL_BINARY_OP(
    M128D, double, -, operator-, operator-=)
VSMC_DEFINE_UTILITY_SIMD_REAL_BINARY_OP(
    M128D, double, *, operator*, operator*=)
VSMC_DEFINE_UTILITY_SIMD_REAL_BINARY_OP(
    M128D, double, /, operator/, operator/=)

namespace internal
{

template <typename RealType>
class M128TypeTrait;

template <>
class M128TypeTrait<float>
{
    public:
    using type = M128;
};

template <>
class M128TypeTrait<double>
{
    public:
    using type = M128D;
};

} // namespace vsmc::internal

/// \brief floating point SSE2 type
/// \ingroup SIMD
template <typename T>
using M128Type = typename std::conditional<std::is_integral<T>::value,
    M128I<T>, typename internal::M128TypeTrait<T>::type>::type;

#endif // VSMC_HAS_SSE2

#if VSMC_HAS_AVX2
#include <immintrin.h>

/// \brief Using `__mm256i` as integer vector
/// \ingroup SIMD
template <typename IntType = __m256i>
class M256I
{
    public:
    using value_type = IntType;

    M256I() = default;

    M256I(const __m256i &value) : value_(value) {}

    template <typename T>
    M256I(const M256I<T> &other) : value_(other.value())
    {
    }

    template <typename T>
    M256I<IntType> &operator=(const M256I<T> &other)
    {
        value_ = other.value_;

        return *this;
    }

    static constexpr std::size_t size()
    {
        return sizeof(__m256i) / sizeof(IntType);
    }

    __m256i &value() { return value_; }
    const __m256i &value() const { return value_; }

    __m256i *data() { return &value_; }
    const __m256i *data() const { return &value_; }

    template <typename T>
    void load_a(const T *mem)
    {
        value_ = _mm256_load_si256(reinterpret_cast<const __m256i *>(mem));
    }

    template <typename T>
    void load_u(const T *mem)
    {
        value_ = _mm256_loadu_si256(reinterpret_cast<const __m256i *>(mem));
    }

    template <typename T>
    void load(const T *mem)
    {
        reinterpret_cast<std::uintptr_t>(mem) % 32 == 0 ? load_a(mem) :
                                                          load_u(mem);
    }

    template <typename T>
    void store_a(T *mem) const
    {
        _mm256_store_si256(reinterpret_cast<__m256i *>(mem), value_);
    }

    template <typename T>
    void store_u(T *mem) const
    {
        _mm256_storeu_si256(reinterpret_cast<__m256i *>(mem), value_);
    }

    template <typename T>
    void store(T *mem) const
    {
        reinterpret_cast<std::uintptr_t>(mem) % 32 == 0 ? store_a(mem) :
                                                          store_u(mem);
    }

    void set0() { value_ = _mm256_setzero_si256(); }

    template <typename T>
    void set1(T n)
    {
        value_ = set1(
            n, std::integral_constant<int, std::numeric_limits<T>::digits>());
    }

    template <typename T>
    void set(T e3, T e2, T e1, T e0)
    {
        value_ = _mm256_set_epi64x(static_cast<VSMC_INT64>(e3),
            static_cast<VSMC_INT64>(e2), static_cast<VSMC_INT64>(e1),
            static_cast<VSMC_INT64>(e0));
    }

    template <typename T>
    void set(T e7, T e6, T e5, T e4, T e3, T e2, T e1, T e0)
    {
        value_ = _mm256_set_epi32(static_cast<int>(e7), static_cast<int>(e6),
            static_cast<int>(e5), static_cast<int>(e4), static_cast<int>(e3),
            static_cast<int>(e2), static_cast<int>(e1), static_cast<int>(e0));
    }

    template <typename T>
    void set(T e15, T e14, T e13, T e12, T e11, T e10, T e9, T e8, T e7, T e6,
        T e5, T e4, T e3, T e2, T e1, T e0)
    {
        value_ =
            _mm256_set_epi16(static_cast<short>(e15), static_cast<short>(e14),
                static_cast<short>(e13), static_cast<short>(e12),
                static_cast<short>(e11), static_cast<short>(e10),
                static_cast<short>(e9), static_cast<short>(e8),
                static_cast<short>(e7), static_cast<short>(e6),
                static_cast<short>(e5), static_cast<short>(e4),
                static_cast<short>(e3), static_cast<short>(e2),
                static_cast<short>(e1), static_cast<short>(e0));
    }

    template <typename T>
    void set(T e31, T e30, T e29, T e28, T e27, T e26, T e25, T e24, T e23,
        T e22, T e21, T e20, T e19, T e18, T e17, T e16, T e15, T e14, T e13,
        T e12, T e11, T e10, T e9, T e8, T e7, T e6, T e5, T e4, T e3, T e2,
        T e1, T e0)
    {
        value_ =
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
                static_cast<char>(e1), static_cast<char>(e0));
    }

    private:
    __m256i value_;

    template <typename T>
    __m256i set1(T n, std::integral_constant<int, 8>)
    {
        return _mm256_set1_epi8(static_cast<char>(n));
    }

    template <typename T>
    __m256i set1(T n, std::integral_constant<int, 16>)
    {
        return _mm256_set1_epi16(static_cast<short>(n));
    }

    template <typename T>
    __m256i set1(T n, std::integral_constant<int, 32>)
    {
        return _mm256_set1_epi32(static_cast<int>(n));
    }

    template <typename T>
    __m256i set1(T n, std::integral_constant<int, 64>)
    {
        return _mm256_set1_epi64x(static_cast<long long>(n));
    }
}; // class M256I

namespace internal
{

template <typename T>
inline M256I<T> m256i_add(
    const M256I<T> &a, const M256I<T> &b, std::integral_constant<int, 8>)
{
    return M256I<T>(_mm256_add_epi8(a.value(), b.value()));
}

template <typename T>
inline M256I<T> m256i_add(
    const M256I<T> &a, const M256I<T> &b, std::integral_constant<int, 16>)
{
    return M256I<T>(_mm256_add_epi16(a.value(), b.value()));
}

template <typename T>
inline M256I<T> m256i_add(
    const M256I<T> &a, const M256I<T> &b, std::integral_constant<int, 32>)
{
    return M256I<T>(_mm256_add_epi32(a.value(), b.value()));
}

template <typename T>
inline M256I<T> m256i_add(
    const M256I<T> &a, const M256I<T> &b, std::integral_constant<int, 64>)
{
    return M256I<T>(_mm256_add_epi64(a.value(), b.value()));
}

template <typename T>
inline M256I<T> m256i_sub(
    const M256I<T> &a, const M256I<T> &b, std::integral_constant<int, 8>)
{
    return M256I<T>(_mm256_sub_epi8(a.value(), b.value()));
}

template <typename T>
inline M256I<T> m256i_sub(
    const M256I<T> &a, const M256I<T> &b, std::integral_constant<int, 16>)
{
    return M256I<T>(_mm256_sub_epi16(a.value(), b.value()));
}

template <typename T>
inline M256I<T> m256i_sub(
    const M256I<T> &a, const M256I<T> &b, std::integral_constant<int, 32>)
{
    return M256I<T>(_mm256_sub_epi32(a.value(), b.value()));
}

template <typename T>
inline M256I<T> m256i_sub(
    const M256I<T> &a, const M256I<T> &b, std::integral_constant<int, 64>)
{
    return M256I<T>(_mm256_sub_epi64(a.value(), b.value()));
}

template <typename T>
inline M256I<T> m256i_slli(
    const M256I<T> &a, int imm8, std::integral_constant<int, 8>)
{
    return M256I<T>(_mm256_slli_epi8(a.value(), imm8));
}

template <typename T>
inline M256I<T> m256i_slli(
    const M256I<T> &a, int imm8, std::integral_constant<int, 16>)
{
    return M256I<T>(_mm256_slli_epi16(a.value(), imm8));
}

template <typename T>
inline M256I<T> m256i_slli(
    const M256I<T> &a, int imm8, std::integral_constant<int, 32>)
{
    return M256I<T>(_mm256_slli_epi32(a.value(), imm8));
}

template <typename T>
inline M256I<T> m256i_slli(
    const M256I<T> &a, int imm8, std::integral_constant<int, 64>)
{
    return M256I<T>(_mm256_slli_epi64(a.value(), imm8));
}

template <typename T>
inline M256I<T> m256i_srli(
    const M256I<T> &a, int imm8, std::integral_constant<int, 8>)
{
    return M256I<T>(_mm256_srli_epi8(a.value(), imm8));
}

template <typename T>
inline M256I<T> m256i_srli(
    const M256I<T> &a, int imm8, std::integral_constant<int, 16>)
{
    return M256I<T>(_mm256_srli_epi16(a.value(), imm8));
}

template <typename T>
inline M256I<T> m256i_srli(
    const M256I<T> &a, int imm8, std::integral_constant<int, 32>)
{
    return M256I<T>(_mm256_srli_epi32(a.value(), imm8));
}

template <typename T>
inline M256I<T> m256i_srli(
    const M256I<T> &a, int imm8, std::integral_constant<int, 64>)
{
    return M256I<T>(_mm256_srli_epi64(a.value(), imm8));
}

} // namespace vsmc::internal

template <typename T>
inline bool operator==(const M256I<T> &a, const M256I<T> &b)
{
    std::array<std::uint64_t, 4> sa;
    std::array<std::uint64_t, 4> sb;
    a.store_u(sa.data());
    b.store_u(sb.data());

    return sa == sb;
}

template <typename T>
inline bool operator!=(const M256I<T> &a, const M256I<T> &b)
{
    return !(a == b);
}

template <typename CharT, typename Traits, typename T>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const M256I<T> &a)
{
    if (!os)
        return os;

    std::array<T, M256I<T>::size()> sa;
    a.store_u(sa.data());
    os << sa;

    return os;
}

template <typename CharT, typename Traits, typename T>
inline std::basic_istream<CharT, Traits> &operator>>(
    std::basic_istream<CharT, Traits> &is, M256I<T> &a)
{
    if (!is)
        return is;

    std::array<T, M256I<T>::size()> sa;
    is >> sa;

    if (static_cast<bool>(is))
        a.load_u(sa.data());

    return is;
}

template <typename T>
inline M256I<T> operator+(const M256I<T> &a, const M256I<T> &b)
{
    return internal::m256i_add(
        a, b, std::integral_constant<int, std::numeric_limits<T>::digits>());
}

template <typename T>
inline M256I<T> operator-(const M256I<T> &a, const M256I<T> &b)
{
    return internal::m256i_sub(
        a, b, std::integral_constant<int, std::numeric_limits<T>::digits>());
}

template <typename T>
inline M256I<T> operator&(const M256I<T> &a, const M256I<T> &b)
{
    return M256I<T>(_mm256_and_si256(a.value(), b.value()));
}

template <typename T>
inline M256I<T> operator|(const M256I<T> &a, const M256I<T> &b)
{
    return M256I<T>(_mm256_or_si256(a.value(), b.value()));
}

template <typename T>
inline M256I<T> operator^(const M256I<T> &a, const M256I<T> &b)
{
    return M256I<T>(_mm256_xor_si256(a.value(), b.value()));
}

template <typename T>
inline M256I<T> operator<<(const M256I<T> &a, int imm8)
{
    return internal::m256i_slli(a, imm8,
        std::integral_constant<int, std::numeric_limits<T>::digits>());
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
    return internal::m256i_srli(a, imm8,
        std::integral_constant<int, std::numeric_limits<T>::digits>());
}

template <typename T>
inline M256I<T> operator>>=(M256I<T> &a, int imm8)
{
    a = a << imm8;

    return a;
}

VSMC_DEFINE_UTILITY_SIMD_INTEGER_BINARY_OP(
    M256I<T>, T, +, operator+, operator+=)
VSMC_DEFINE_UTILITY_SIMD_INTEGER_BINARY_OP(
    M256I<T>, T, -, operator-, operator-=)
VSMC_DEFINE_UTILITY_SIMD_INTEGER_BINARY_OP(
    M256I<T>, T, &, operator&, operator&=)
VSMC_DEFINE_UTILITY_SIMD_INTEGER_BINARY_OP(
    M256I<T>, T, |, operator|, operator|=)
VSMC_DEFINE_UTILITY_SIMD_INTEGER_BINARY_OP(
    M256I<T>, T, ^, operator^, operator^=)

/// \brief `__m256`
/// \ingroup SIMD
class M256
{
    public:
    M256() = default;

    M256(const __m256 &value) : value_(value) {}

    static constexpr std::size_t size() { return 8; }

    __m256 &value() { return value_; }
    const __m256 &value() const { return value_; }

    __m256 *data() { return &value_; }
    const __m256 *data() const { return &value_; }

    template <typename T>
    void load_a(const T *mem)
    {
        value_ = _mm256_load_ps(reinterpret_cast<const float *>(mem));
    }

    template <typename T>
    void load_u(const T *mem)
    {
        value_ = _mm256_loadu_ps(reinterpret_cast<const float *>(mem));
    }

    template <typename T>
    void load(const T *mem)
    {
        reinterpret_cast<std::uintptr_t>(mem) % 32 == 0 ? load_a(mem) :
                                                          load_u(mem);
    }

    template <typename T>
    void store_a(T *mem) const
    {
        _mm256_store_ps(reinterpret_cast<float *>(mem), value_);
    }

    template <typename T>
    void store_u(T *mem) const
    {
        _mm256_storeu_ps(reinterpret_cast<float *>(mem), value_);
    }

    template <typename T>
    void store(T *mem) const
    {
        reinterpret_cast<std::uintptr_t>(mem) % 32 == 0 ? store_a(mem) :
                                                          store_u(mem);
    }

    void set0() { value_ = _mm256_setzero_ps(); }

    void set1(float e) { value_ = _mm256_set1_ps(e); }

    void set(float e7, float e6, float e5, float e4, float e3, float e2,
        float e1, float e0)
    {
        value_ = _mm256_set_ps(e7, e6, e5, e4, e3, e2, e1, e0);
    }

    private:
    __m256 value_;
}; // class M256

inline bool operator==(const M256 &a, const M256 &b)
{
    std::array<float, 8> sa;
    std::array<float, 8> sb;
    a.store_u(sa.data());
    b.store_u(sb.data());

    return sa == sb;
}

inline bool operator!=(const M256 &a, const M256 &b) { return !(a == b); }

template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const M256 &a)
{
    if (!os)
        return os;

    std::array<float, 8> sa;
    a.store_u(sa.data());
    os << sa;

    return os;
}

template <typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits> &operator>>(
    std::basic_istream<CharT, Traits> &is, M256 &a)
{
    if (!is)
        return is;

    std::array<float, 8> sa;
    is >> sa;

    if (static_cast<bool>(is))
        a.load_u(sa.data());

    return is;
}

inline M256 operator+(const M256 &a, const M256 &b)
{
    return M256(_mm256_add_ps(a.value(), b.value()));
}

inline M256 operator-(const M256 &a, const M256 &b)
{
    return M256(_mm256_sub_ps(a.value(), b.value()));
}

inline M256 operator*(const M256 &a, const M256 &b)
{
    return M256(_mm256_mul_ps(a.value(), b.value()));
}

inline M256 operator/(const M256 &a, const M256 &b)
{
    return M256(_mm256_div_ps(a.value(), b.value()));
}

VSMC_DEFINE_UTILITY_SIMD_REAL_BINARY_OP(M256, float, +, operator+, operator+=)
VSMC_DEFINE_UTILITY_SIMD_REAL_BINARY_OP(M256, float, -, operator-, operator-=)
VSMC_DEFINE_UTILITY_SIMD_REAL_BINARY_OP(M256, float, *, operator*, operator*=)
VSMC_DEFINE_UTILITY_SIMD_REAL_BINARY_OP(M256, float, /, operator/, operator/=)

/// \brief `__m256d`
/// \ingroup SIMD
class M256D
{
    public:
    M256D() = default;

    M256D(const __m256d &value) : value_(value) {}

    static constexpr std::size_t size() { return 4; }

    __m256d &value() { return value_; }
    const __m256d &value() const { return value_; }

    __m256d *data() { return &value_; }
    const __m256d *data() const { return &value_; }

    template <typename T>
    void load_a(const T *mem)
    {
        value_ = _mm256_load_pd(reinterpret_cast<const double *>(mem));
    }

    template <typename T>
    void load_u(const T *mem)
    {
        value_ = _mm256_loadu_pd(reinterpret_cast<const double *>(mem));
    }

    template <typename T>
    void load(const T *mem)
    {
        reinterpret_cast<std::uintptr_t>(mem) % 32 == 0 ? load_a(mem) :
                                                          load_u(mem);
    }

    template <typename T>
    void store_a(T *mem) const
    {
        _mm256_store_pd(reinterpret_cast<double *>(mem), value_);
    }

    template <typename T>
    void store_u(T *mem) const
    {
        _mm256_storeu_pd(reinterpret_cast<double *>(mem), value_);
    }

    template <typename T>
    void store(T *mem) const
    {
        reinterpret_cast<std::uintptr_t>(mem) % 32 == 0 ? store_a(mem) :
                                                          store_u(mem);
    }

    void set0() { value_ = _mm256_setzero_pd(); }

    void set1(double e) { value_ = _mm256_set1_pd(e); }

    void set(double e3, double e2, double e1, double e0)
    {
        value_ = _mm256_set_pd(e3, e2, e1, e0);
    }

    private:
    __m256d value_;
}; // class M256D

inline bool operator==(const M256D &a, const M256D &b)
{
    std::array<double, 4> sa;
    std::array<double, 4> sb;
    a.store_u(sa.data());
    b.store_u(sb.data());

    return sa == sb;
}

inline bool operator!=(const M256D &a, const M256D &b) { return !(a == b); }

template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const M256D &a)
{
    if (!os)
        return os;

    std::array<double, 4> sa;
    a.store_u(sa.data());
    os << sa;

    return os;
}

template <typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits> &operator>>(
    std::basic_istream<CharT, Traits> &is, M256D &a)
{
    if (!is)
        return is;

    std::array<double, 4> sa;
    is >> sa;

    if (static_cast<bool>(is))
        a.load_u(sa.data());

    return is;
}

inline M256D operator+(const M256D &a, const M256D &b)
{
    return M256D(_mm256_add_pd(a.value(), b.value()));
}

inline M256D operator-(const M256D &a, const M256D &b)
{
    return M256D(_mm256_sub_pd(a.value(), b.value()));
}

inline M256D operator*(const M256D &a, const M256D &b)
{
    return M256D(_mm256_mul_pd(a.value(), b.value()));
}

inline M256D operator/(const M256D &a, const M256D &b)
{
    return M256D(_mm256_div_pd(a.value(), b.value()));
}

VSMC_DEFINE_UTILITY_SIMD_REAL_BINARY_OP(
    M256D, double, +, operator+, operator+=)
VSMC_DEFINE_UTILITY_SIMD_REAL_BINARY_OP(
    M256D, double, -, operator-, operator-=)
VSMC_DEFINE_UTILITY_SIMD_REAL_BINARY_OP(
    M256D, double, *, operator*, operator*=)
VSMC_DEFINE_UTILITY_SIMD_REAL_BINARY_OP(
    M256D, double, /, operator/, operator/=)

namespace internal
{

template <typename RealType>
class M256TypeTrait;

template <>
class M256TypeTrait<float>
{
    public:
    using type = M256;
};

template <>
class M256TypeTrait<double>
{
    public:
    using type = M256D;
};

} // namespace vsmc::internal

/// \brief floating point SSE2 type
/// \ingroup SIMD
template <typename T>
using M256Type = typename std::conditional<std::is_integral<T>::value,
    M256I<T>, typename internal::M256TypeTrait<T>::type>::type;

#endif // VSMC_HAS_AVX2

} // namespace vsmc

#endif // VSMC_UTILITY_SIMD_HPP
