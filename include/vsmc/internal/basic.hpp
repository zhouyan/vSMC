//============================================================================
// vSMC/include/vsmc/internal/basic.hpp
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
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_INTERNAL_BASIC_HPP
#define VSMC_INTERNAL_BASIC_HPP

#include <vsmc/internal/config.h>

#include <vsmc/internal/assert.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/forward.hpp>
#include <vsmc/internal/traits.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <functional>
#include <future>
#include <initializer_list>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <new>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace vsmc
{

namespace internal
{

#ifdef VSMC_CLANG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wfloat-equal"
#endif

template <typename T>
inline bool is_equal(const T &a, const T &b)
{
    return a == b;
}

#ifdef VSMC_CLANG
#pragma clang diagnostic pop
#endif

template <typename T>
inline bool is_zero(const T &a)
{
    return is_equal(a, static_cast<T>(0));
}

template <typename T>
inline bool is_one(const T &a)
{
    return is_equal(a, static_cast<T>(1));
}

template <typename T>
inline bool is_nullptr(T ptr, std::true_type)
{
    return ptr == nullptr;
}

template <typename T>
inline bool is_nullptr(T, std::false_type)
{
    return false;
}

template <typename T>
inline bool is_nullptr(T ptr)
{
    return is_nullptr(ptr, std::is_pointer<T>());
}

template <unsigned long long U,
    int N = std::numeric_limits<unsigned long long>::digits - 1>
class Log2L
{
    static constexpr unsigned long long M = 1ULL << N;

    public:
    static constexpr int value = M <= U ? N : Log2L<U, N - 1>::value;
}; // class Log2L

template <unsigned long long U>
class Log2L<U, 0>
{
    public:
    static constexpr int value = 0;
}; // class Log2L

template <int N>
class Log2L<0, N>
{
    public:
    static constexpr int value = N + 1;
}; // class Log2L

template <typename UIntType, UIntType U>
class Log2 : public Log2L<U, std::numeric_limits<UIntType>::digits - 1>
{
}; // class Log2

template <typename T>
class BufferSize : public std::integral_constant<std::size_t, 8192 / sizeof(T)>
{
}; // class BufferSize;

template <std::size_t, typename CharT, typename Traits, typename T,
    std::size_t N>
inline void ostream_ary_space(std::basic_ostream<CharT, Traits> &,
    const std::array<T, N> &, std::false_type)
{
}

template <std::size_t, typename CharT, typename Traits, typename T,
    std::size_t N>
inline void ostream_ary_space(std::basic_ostream<CharT, Traits> &os,
    const std::array<T, N> &, std::true_type)
{
    os << ' ';
}

template <std::size_t, typename CharT, typename Traits, typename T,
    std::size_t N>
inline void ostream_ary(std::basic_ostream<CharT, Traits> &,
    const std::array<T, N> &, std::false_type)
{
}

template <std::size_t K, typename CharT, typename Traits, typename T,
    std::size_t N>
inline void ostream_ary(std::basic_ostream<CharT, Traits> &os,
    const std::array<T, N> &ary, std::true_type)
{
    os << std::get<K>(ary);
    ostream_ary_space<K>(os, ary, std::integral_constant<bool, K + 1 != N>());
    ostream_ary<K + 1>(os, ary, std::integral_constant<bool, K + 1 < N>());
}

template <std::size_t, typename CharT, typename Traits, typename T,
    std::size_t N>
inline void istream_ary(
    std::basic_istream<CharT, Traits> &, std::array<T, N> &, std::false_type)
{
}

template <std::size_t K, typename CharT, typename Traits, typename T,
    std::size_t N>
inline void istream_ary(std::basic_istream<CharT, Traits> &is,
    std::array<T, N> &ary, std::true_type)
{
    is >> std::ws >> std::get<K>(ary);
    istream_ary<K + 1>(is, ary, std::integral_constant<bool, K + 1 < N>());
}

template <typename CharT, typename Traits, typename T, typename Alloc>
inline void ostream_vec(
    std::basic_ostream<CharT, Traits> &os, const std::vector<T, Alloc> &vec)
{
    os << vec.size();
    if (!os)
        return;

    for (const auto &v : vec)
        os << ' ' << v;
}

template <typename CharT, typename Traits, typename T, typename Alloc>
inline void istream_vec(
    std::basic_istream<CharT, Traits> &is, std::vector<T, Alloc> &vec)
{
    std::size_t n = 0;
    is >> n;
    if (!is)
        return;

    vec.resize(n);
    for (std::size_t i = 0; i != n; ++i)
        is >> std::ws >> vec[i];
}

template <typename CharT, typename Traits, typename T, std::size_t N>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const std::array<T, N> &ary)
{
    if (!os)
        return os;

    ostream_ary<0>(os, ary, std::integral_constant<bool, 0 < N>());

    return os;
}

template <typename CharT, typename Traits, typename T, std::size_t N>
inline std::basic_istream<CharT, Traits> &operator>>(
    std::basic_istream<CharT, Traits> &is, std::array<T, N> &ary)
{
    if (!is)
        return is;

    std::array<T, N> tmp;
    istream_ary<0>(is, tmp, std::integral_constant<bool, 0 < N>());
    if (static_cast<bool>(is))
        ary = std::move(tmp);

    return is;
}

template <typename CharT, typename Traits, typename T, typename Alloc>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const std::vector<T, Alloc> &vec)
{
    if (!os)
        return os;

    ostream_vec(os, vec);

    return os;
}

template <typename CharT, typename Traits, typename T, typename Alloc>
inline std::basic_istream<CharT, Traits> &operator>>(
    std::basic_istream<CharT, Traits> &is, std::vector<T, Alloc> &vec)
{
    if (!is)
        return is;

    std::vector<T, Alloc> tmp;
    istream_vec(is, tmp);
    if (static_cast<bool>(is))
        vec = std::move(tmp);

    return is;
}

} // namespace vsmc::internal

template <typename CharT, typename Traits, typename T, std::size_t N>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const std::array<T, N> &ary)
{
    if (!os)
        return os;

    internal::ostream_ary<0>(os, ary, std::integral_constant<bool, 0 < N>());

    return os;
}

template <typename CharT, typename Traits, typename T, std::size_t N>
inline std::basic_istream<CharT, Traits> &operator>>(
    std::basic_istream<CharT, Traits> &is, std::array<T, N> &ary)
{
    if (!is)
        return is;

    std::array<T, N> tmp;
    internal::istream_ary<0>(is, tmp, std::integral_constant<bool, 0 < N>());
    if (static_cast<bool>(is))
        ary = std::move(tmp);

    return is;
}

template <typename CharT, typename Traits, typename T, typename Alloc>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const std::vector<T, Alloc> &vec)
{
    if (!os)
        return os;

    internal::ostream_vec(os, vec);

    return os;
}

template <typename CharT, typename Traits, typename T, typename Alloc>
inline std::basic_istream<CharT, Traits> &operator>>(
    std::basic_istream<CharT, Traits> &is, std::vector<T, Alloc> &vec)
{
    if (!is)
        return is;

    std::vector<T, Alloc> tmp;
    internal::istream_vec(is, tmp);
    if (static_cast<bool>(is))
        vec = std::move(tmp);

    return is;
}

} // namespace vsmc

#endif // VSMC_INTERNAL_BASIC_HPP
