//============================================================================
// vSMC/include/vsmc/internal/common.hpp
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

#ifndef VSMC_INTERNAL_COMMON_HPP
#define VSMC_INTERNAL_COMMON_HPP

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/assert.hpp>
#include <vsmc/internal/forward.hpp>
#include <vsmc/internal/traits.hpp>

#include <vsmc/math/cblas.hpp>
#include <vsmc/math/constants.hpp>
#include <vsmc/math/vmath.hpp>

#include <algorithm>
#include <array>
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
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace vsmc
{

namespace internal
{

template <typename UIntType>
inline std::string itos(UIntType i, std::true_type)
{
    if (i == 0)
        return std::string("0");

    char str[24] = {0};
    std::size_t n = 0;
    while (i > 0) {
        str[n++] = '0' + i % 10;
        i /= 10;
    }
    std::reverse(str, str + n);

    return std::string(str);
}

template <typename IntType>
inline std::string itos(IntType i, std::false_type)
{
    typedef typename std::make_unsigned<IntType>::type uint_type;

    if (i < 0)
        return "-" + itos(static_cast<uint_type>(-i));

    return itos(static_cast<uint_type>(i));
}

template <typename IntType>
inline std::string itos(IntType i)
{
    return itos(i, std::is_unsigned<IntType>());
}

} // namespace vsmc::internal

template <typename CharT, typename Traits, typename T, std::size_t N>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const std::array<T, N> &ary)
{
    if (!os.good())
        return os;

    for (std::size_t i = 0; i < N - 1; ++i)
        os << ary[i] << ' ';
    os << ary[N - 1];

    return os;
}

template <typename CharT, typename Traits, typename T, std::size_t N>
inline std::basic_istream<CharT, Traits> &operator>>(
    std::basic_istream<CharT, Traits> &is, std::array<T, N> &ary)
{
    if (!is.good())
        return is;

    std::array<T, N> ary_tmp;
    for (std::size_t i = 0; i != N; ++i)
        is >> std::ws >> ary_tmp[i];

    if (is.good())
        ary = std::move(ary_tmp);

    return is;
}

} // namespace vsmc

#endif // VSMC_INTERNAL_COMMON_HPP
