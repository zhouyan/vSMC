//============================================================================
// vSMC/include/vsmc/utility/simd.hpp
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

#ifndef VSMC_UTILITY_SIMD_HPP
#define VSMC_UTILITY_SIMD_HPP

#include <vsmc/internal/common.hpp>

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
    M128I(T n,
        typename std::enable_if<std::is_integral<T>::value>::type * = nullptr)
        : value_(set1(n, std::integral_constant<std::size_t, sizeof(T)>()))
    {
    }

    template <typename T>
    M128I(const M128I<T> &other)
        : value_(other.value())
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
        value_ = set1(n, std::integral_constant<std::size_t, sizeof(T)>());
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

namespace internal
{

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

} // namespace internal

inline bool operator==(const M128I<> &a, const M128I<> &b)
{
    std::array<std::uint64_t, 2> sa;
    std::array<std::uint64_t, 2> sb;
    a.store_u(sa.data());
    b.store_u(sb.data());

    return sa == sb;
}

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

template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const __m128i &a)
{
    if (!os.good())
        return os;

    std::array<std::uint64_t, 2> sa;
    M128I<> ma(a);
    ma.store_u(sa.data());
    os << sa;

    return os;
}

template <typename CharT, typename Traits, typename T>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const M128I<T> &a)
{
    if (!os.good())
        return os;

    std::array<T, M128I<T>::size()> sa;
    a.store_u(sa.data());
    os << sa;

    return os;
}

template <typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits> &operator>>(
    std::basic_istream<CharT, Traits> &is, __m128i &a)
{
    if (!is.good())
        return is;

    std::array<std::uint64_t, 2> sa;
    is >> sa;

    if (is.good()) {
        M128I<> ma;
        ma.load_u(sa.data());
        a = ma.value();
    }

    return is;
}

template <typename CharT, typename Traits, typename T>
inline std::basic_istream<CharT, Traits> &operator>>(
    std::basic_istream<CharT, Traits> &is, M128I<T> &a)
{
    if (!is.good())
        return is;

    std::array<T, M128I<T>::size()> sa;
    is >> sa;

    if (is.good())
        a.load_u(sa.data());

    return is;
}

template <typename T>
inline M128I<T> operator+(const M128I<T> &a, const M128I<T> &b)
{
    return internal::m128i_add(
        a, b, std::integral_constant<std::size_t, sizeof(T)>());
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
    return internal::m128i_sub(
        a, b, std::integral_constant<std::size_t, sizeof(T)>());
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
    return internal::m128i_slli(
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
    return internal::m128i_srli(
        a, imm8, std::integral_constant<std::size_t, sizeof(T)>());
}

template <typename T>
inline M128I<T> operator>>=(M128I<T> &a, int imm8)
{
    a = a << imm8;

    return a;
}

/// \brief `__m128`
/// \ingroup SIMD
class M128
{
    public:
    M128() = default;

    M128(const __m128 &value) : value_(value) {}

    M128(float e) : value_(_mm_set1_ps(e)) {}

    M128(float e3, float e2, float e1, float e0)
        : value_(_mm_set_ps(e3, e2, e1, e0))
    {
    }

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

inline M128 operator+(const M128 &a, const M128 &b)
{
    return M128(_mm_add_ps(a.value(), b.value()));
}

inline M128 &operator+=(M128 &a, const M128 &b)
{
    a = a + b;

    return a;
}

inline M128 operator-(const M128 &a, const M128 &b)
{
    return M128(_mm_sub_ps(a.value(), b.value()));
}

inline M128 &operator-=(M128 &a, const M128 &b)
{
    a = a - b;

    return a;
}

inline M128 operator*(const M128 &a, const M128 &b)
{
    return M128(_mm_mul_ps(a.value(), b.value()));
}

inline M128 &operator*=(M128 &a, const M128 &b)
{
    a = a * b;

    return a;
}

inline M128 operator/(const M128 &a, const M128 &b)
{
    return M128(_mm_div_ps(a.value(), b.value()));
}

inline M128 &operator/=(M128 &a, const M128 &b)
{
    a = a / b;

    return a;
}

namespace internal
{

template <typename T>
inline void convert_m128i_ps(
    const M128I<T> &n, M128 *f, std::integral_constant<std::size_t, 1>)
{
    alignas(16) T a[16];
    alignas(16) float b[16];
    n.store_a(a);
    for (std::size_t i = 0; i != 16; ++i)
        b[i] = static_cast<float>(a[i]);
    f[0].load_a(b + 0);
    f[1].load_a(b + 4);
    f[2].load_a(b + 8);
    f[3].load_a(b + 12);
}

template <typename T>
inline void convert_m128i_ps(
    const M128I<T> &n, M128 *f, std::integral_constant<std::size_t, 2>)
{
    alignas(16) T a[8];
    alignas(16) float b[8];
    n.store_a(a);
    b[0] = static_cast<float>(a[0]);
    b[1] = static_cast<float>(a[1]);
    b[2] = static_cast<float>(a[2]);
    b[3] = static_cast<float>(a[3]);
    b[4] = static_cast<float>(a[4]);
    b[5] = static_cast<float>(a[5]);
    b[6] = static_cast<float>(a[6]);
    b[7] = static_cast<float>(a[7]);
    f[0].load_a(b + 0);
    f[1].load_a(b + 4);
}

template <typename T>
inline void convert_m128i_ps(
    const M128I<T> &n, M128 *f, std::integral_constant<std::size_t, 4>)
{
    alignas(16) T a[4];
    alignas(16) float b[4];
    n.store_a(a);
    b[0] = static_cast<float>(a[0]);
    b[1] = static_cast<float>(a[1]);
    b[2] = static_cast<float>(a[2]);
    b[3] = static_cast<float>(a[3]);
    f[0].load_a(b);
}

template <typename T>
inline void convert_m128i_ps(
    const M128I<T> &n, M128 *f, std::integral_constant<std::size_t, 8>)
{
    alignas(16) T a[2];
    alignas(16) float b[2];
    n.store_a(a);
    b[0] = static_cast<float>(a[0]);
    b[1] = static_cast<float>(a[1]);
    f[0].set(0, 0, b[1], b[0]);
}

} // namespace vsmc::internal

template <typename T>
inline void convert_m128i_ps(const M128I<T> &n, M128 *f)
{
    internal::convert_m128i_ps(
        n, f, std::integral_constant<std::size_t, sizeof(T)>());
}

/// \brief `__m128d`
/// \ingroup SIMD
class M128D
{
    public:
    M128D() = default;

    M128D(const __m128d &value) : value_(value) {}

    M128D(double e) : value_(_mm_set1_pd(e)) {}

    M128D(double e1, double e0) : value_(_mm_set_pd(e1, e0)) {}

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
}; // class M128

inline M128D operator+(const M128D &a, const M128D &b)
{
    return M128D(_mm_add_pd(a.value(), b.value()));
}

inline M128D &operator+=(M128D &a, const M128D &b)
{
    a = a + b;

    return a;
}

inline M128D operator-(const M128D &a, const M128D &b)
{
    return M128D(_mm_sub_pd(a.value(), b.value()));
}

inline M128D &operator-=(M128D &a, const M128D &b)
{
    a = a - b;

    return a;
}

inline M128D operator*(const M128D &a, const M128D &b)
{
    return M128D(_mm_mul_pd(a.value(), b.value()));
}

inline M128D &operator*=(M128D &a, const M128D &b)
{
    a = a * b;

    return a;
}

inline M128D operator/(const M128D &a, const M128D &b)
{
    return M128D(_mm_div_pd(a.value(), b.value()));
}

inline M128D &operator/=(M128D &a, const M128D &b)
{
    a = a / b;

    return a;
}

namespace internal
{

template <typename T>
inline void convert_m128i_pd(
    const M128I<T> &n, M128D *f, std::integral_constant<std::size_t, 1>)
{
    alignas(16) T a[16];
    alignas(16) double b[16];
    n.store_a(a);
    for (std::size_t i = 0; i != 16; ++i)
        b[i] = static_cast<double>(a[i]);
    f[0].load_a(b + 0);
    f[1].load_a(b + 2);
    f[2].load_a(b + 4);
    f[3].load_a(b + 6);
    f[4].load_a(b + 8);
    f[5].load_a(b + 10);
    f[6].load_a(b + 12);
    f[7].load_a(b + 14);
}

template <typename T>
inline void convert_m128i_pd(
    const M128I<T> &n, M128D *f, std::integral_constant<std::size_t, 2>)
{
    alignas(16) T a[8];
    alignas(16) double b[8];
    n.store_a(a);
    b[0] = static_cast<double>(a[0]);
    b[1] = static_cast<double>(a[1]);
    b[2] = static_cast<double>(a[2]);
    b[3] = static_cast<double>(a[3]);
    b[4] = static_cast<double>(a[4]);
    b[5] = static_cast<double>(a[5]);
    b[6] = static_cast<double>(a[6]);
    b[7] = static_cast<double>(a[7]);
    f[0].load_a(b + 0);
    f[1].load_a(b + 2);
    f[2].load_a(b + 4);
    f[3].load_a(b + 6);
}

template <typename T>
inline void convert_m128i_pd(
    const M128I<T> &n, M128D *f, std::integral_constant<std::size_t, 4>)
{
    alignas(16) T a[4];
    alignas(16) double b[4];
    n.store_a(a);
    b[0] = static_cast<double>(a[0]);
    b[1] = static_cast<double>(a[1]);
    b[2] = static_cast<double>(a[2]);
    b[3] = static_cast<double>(a[3]);
    f[0].load_a(b + 0);
    f[1].load_a(b + 2);
}

template <typename T>
inline void convert_m128i_pd(
    const M128I<T> &n, M128D *f, std::integral_constant<std::size_t, 8>)
{
    alignas(16) T a[2];
    alignas(16) double b[2];
    n.store_a(a);
    b[0] = static_cast<double>(a[0]);
    b[1] = static_cast<double>(a[1]);
    f[0].load_a(b);
}

} // namespace vsmc::internal

template <typename T>
inline void convert_m128i_pd(const M128I<T> &n, M128D *f)
{
    internal::convert_m128i_pd(
        n, f, std::integral_constant<std::size_t, sizeof(T)>());
}

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
    M256I(T n,
        typename std::enable_if<std::is_integral<T>::value>::type * = nullptr)
    {
        set1(n);
    }

    template <typename T>
    M256I(const M256I<T> &other)
        : value_(other.value())
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
        value_ = set1(n, std::integral_constant<std::size_t, sizeof(T)>());
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

namespace internal
{

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

} // namespace vsmc::internal

inline bool operator==(const M256I<> &a, const M256I<> &b)
{
    std::array<std::uint64_t, 4> sa;
    std::array<std::uint64_t, 4> sb;
    a.store_u(sa.data());
    b.store_u(sb.data());

    return sa == sb;
}

template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const __m256i &a)
{
    if (!os.good())
        return os;

    std::array<std::uint64_t, 4> sa;
    M256I<> ma(a);
    ma.store_u(sa.data());
    os << sa;

    return os;
}

template <typename CharT, typename Traits, typename T>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const M256I<T> &a)
{
    if (!os.good())
        return os;

    std::array<T, M256I<T>::size()> sa;
    a.store_u(sa.data());
    os << sa;

    return os;
}

template <typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits> &operator>>(
    std::basic_istream<CharT, Traits> &is, __m256i &a)
{
    if (!is.good())
        return is;

    std::array<std::uint64_t, 4> sa;
    is >> sa;

    if (is.good()) {
        M256I<> ma;
        ma.load_u(sa.data());
        a = ma.value();
    }

    return is;
}

template <typename CharT, typename Traits, typename T>
inline std::basic_istream<CharT, Traits> &operator>>(
    std::basic_istream<CharT, Traits> &is, M256I<T> &a)
{
    if (!is.good())
        return is;

    std::array<T, M256I<T>::size()> sa;
    is >> sa;

    if (is.good())
        a.load_u(sa.data());

    return is;
}

template <typename T>
inline M256I<T> operator+(const M256I<T> &a, const M256I<T> &b)
{
    return internal::m256i_add(
        a, b, std::integral_constant<std::size_t, sizeof(T)>());
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
    return internal::m256i_sub(
        a, b, std::integral_constant<std::size_t, sizeof(T)>());
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
    return internal::m256i_slli(
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
    return internal::m256i_srli(
        a, imm8, std::integral_constant<std::size_t, sizeof(T)>());
}

template <typename T>
inline M256I<T> operator>>=(M256I<T> &a, int imm8)
{
    a = a << imm8;

    return a;
}

/// \brief `__m256`
/// \ingroup SIMD
class M256
{
    public:
    M256() = default;

    M256(const __m256 &value) : value_(value) {}

    M256(float e) : value_(_mm256_set1_ps(e)) {}

    M256(float e7, float e6, float e5, float e4, float e3, float e2, float e1,
        float e0)
        : value_(_mm256_set_ps(e7, e6, e5, e4, e3, e2, e1, e0))
    {
    }

    __m256 &value() { return value_; }
    const __m256 &value() const { return value_; }

    __m256 *data() { return &value_; }
    const __m256 *data() const { return &value_; }

    template <typename T>
    M256I<T> m256i() const
    {
        return M256I<T>(_mm256_castsi256_ps(value_));
    }

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

inline M256 operator+(const M256 &a, const M256 &b)
{
    return M256(_mm256_add_ps(a.value(), b.value()));
}

inline M256 &operator+=(M256 &a, const M256 &b)
{
    a = a + b;

    return a;
}

inline M256 operator-(const M256 &a, const M256 &b)
{
    return M256(_mm256_sub_ps(a.value(), b.value()));
}

inline M256 &operator-=(M256 &a, const M256 &b)
{
    a = a - b;

    return a;
}

inline M256 operator*(const M256 &a, const M256 &b)
{
    return M256(_mm256_mul_ps(a.value(), b.value()));
}

inline M256 &operator*=(M256 &a, const M256 &b)
{
    a = a * b;

    return a;
}

inline M256 operator/(const M256 &a, const M256 &b)
{
    return M256(_mm256_div_ps(a.value(), b.value()));
}

inline M256 &operator/=(M256 &a, const M256 &b)
{
    a = a / b;

    return a;
}

namespace internal
{

template <typename T>
inline void convert_m256i_ps(
    const M256I<T> &n, M256 *f, std::integral_constant<std::size_t, 1>)
{
    alignas(32) T a[32];
    alignas(32) float b[32];
    n.store_a(a);
    for (std::size_t i = 0; i != 32; ++i)
        b[i] = static_cast<float>(a[i]);
    f[0].load_a(b + 0);
    f[1].load_a(b + 8);
    f[2].load_a(b + 16);
    f[3].load_a(b + 24);
}

template <typename T>
inline void convert_m256i_ps(
    const M256I<T> &n, M256 *f, std::integral_constant<std::size_t, 2>)
{
    alignas(32) T a[16];
    alignas(32) float b[16];
    n.store_a(a);
    for (std::size_t i = 0; i != 16; ++i)
        b[i] = static_cast<float>(a[i]);
    f[0].load_a(b + 0);
    f[1].load_a(b + 8);
}

template <typename T>
inline void convert_m256i_ps(
    const M256I<T> &n, M256 *f, std::integral_constant<std::size_t, 4>)
{
    alignas(32) T a[8];
    alignas(32) float b[8];
    n.store_a(a);
    b[0] = static_cast<float>(a[0]);
    b[1] = static_cast<float>(a[1]);
    b[2] = static_cast<float>(a[2]);
    b[3] = static_cast<float>(a[3]);
    b[4] = static_cast<float>(a[4]);
    b[5] = static_cast<float>(a[5]);
    b[6] = static_cast<float>(a[6]);
    b[7] = static_cast<float>(a[7]);
    f[0].load_a(b);
}

template <typename T>
inline void convert_m256i_ps(
    const M256I<T> &n, M256 *f, std::integral_constant<std::size_t, 8>)
{
    alignas(32) T a[4];
    alignas(32) float b[4];
    n.store_a(a);
    b[0] = static_cast<float>(a[0]);
    b[1] = static_cast<float>(a[1]);
    b[2] = static_cast<float>(a[2]);
    b[3] = static_cast<float>(a[3]);
    f[0].set(0, 0, 0, 0, b[3], b[2], b[1], b[0]);
}

} // namespace vsmc::internal

template <typename T>
inline void convert_m256i_ps(const M256I<T> &n, M256 *f)
{
    internal::convert_m256i_ps(
        n, f, std::integral_constant<std::size_t, sizeof(T)>());
}

/// \brief `__m256d`
/// \ingroup SIMD
class M256D
{
    public:
    M256D() = default;

    M256D(const __m256d &value) : value_(value) {}

    M256D(double e) : value_(_mm256_set1_pd(e)) {}

    M256D(double e3, double e2, double e1, double e0)
        : value_(_mm256_set_pd(e3, e2, e1, e0))
    {
    }

    __m256d &value() { return value_; }
    const __m256d &value() const { return value_; }

    __m256d *data() { return &value_; }
    const __m256d *data() const { return &value_; }

    template <typename T>
    M256I<T> m256i() const
    {
        return M256I<T>(_mm256_castsi256_pd(value_));
    }

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
}; // class M256

inline M256D operator+(const M256D &a, const M256D &b)
{
    return M256D(_mm256_add_pd(a.value(), b.value()));
}

inline M256D &operator+=(M256D &a, const M256D &b)
{
    a = a + b;

    return a;
}

inline M256D operator-(const M256D &a, const M256D &b)
{
    return M256D(_mm256_sub_pd(a.value(), b.value()));
}

inline M256D &operator-=(M256D &a, const M256D &b)
{
    a = a - b;

    return a;
}

inline M256D operator*(const M256D &a, const M256D &b)
{
    return M256D(_mm256_mul_pd(a.value(), b.value()));
}

inline M256D &operator*=(M256D &a, const M256D &b)
{
    a = a * b;

    return a;
}

inline M256D operator/(const M256D &a, const M256D &b)
{
    return M256D(_mm256_div_pd(a.value(), b.value()));
}

inline M256D &operator/=(M256D &a, const M256D &b)
{
    a = a / b;

    return a;
}

namespace internal
{

template <typename T>
inline void convert_m256i_pd(
    const M256I<T> &n, M256D *f, std::integral_constant<std::size_t, 1>)
{
    alignas(32) T a[32];
    alignas(32) double b[32];
    n.store_a(a);
    for (std::size_t i = 0; i != 32; ++i)
        b[i] = static_cast<double>(a[i]);
    f[0].load_a(b + 0);
    f[1].load_a(b + 8);
    f[2].load_a(b + 16);
    f[3].load_a(b + 24);
}

template <typename T>
inline void convert_m256i_pd(
    const M256I<T> &n, M256D *f, std::integral_constant<std::size_t, 2>)
{
    alignas(32) T a[16];
    alignas(32) double b[16];
    n.store_a(a);
    for (std::size_t i = 0; i != 16; ++i)
        b[i] = static_cast<double>(a[i]);
    f[0].load_a(b + 0);
    f[1].load_a(b + 8);
}

template <typename T>
inline void convert_m256i_pd(
    const M256I<T> &n, M256D *f, std::integral_constant<std::size_t, 4>)
{
    alignas(32) T a[8];
    alignas(32) double b[8];
    n.store_a(a);
    b[0] = static_cast<double>(a[0]);
    b[1] = static_cast<double>(a[1]);
    b[2] = static_cast<double>(a[2]);
    b[3] = static_cast<double>(a[3]);
    b[4] = static_cast<double>(a[4]);
    b[5] = static_cast<double>(a[5]);
    b[6] = static_cast<double>(a[6]);
    b[7] = static_cast<double>(a[7]);
    f[0].load_a(b);
}

template <typename T>
inline void convert_m256i_pd(
    const M256I<T> &n, M256D *f, std::integral_constant<std::size_t, 8>)
{
    alignas(32) T a[2];
    alignas(32) double b[2];
    n.store_a(a);
    b[0] = static_cast<double>(a[0]);
    b[1] = static_cast<double>(a[1]);
    f[0].load_a(b);
}

} // namespace vsmc::internal

template <typename T>
inline void convert_m256i_pd(const M256I<T> &n, M256D *f)
{
    internal::convert_m256i_pd(
        n, f, std::integral_constant<std::size_t, sizeof(T)>());
}

#endif // VSMC_HAS_AVX2

} // namespace vsmc

#endif // VSMC_UTILITY_SIMD_HPP
