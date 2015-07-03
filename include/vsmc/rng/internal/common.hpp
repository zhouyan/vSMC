//============================================================================
// vSMC/include/vsmc/rng/internal/common.hpp
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

#ifndef VSMC_RNG_INTERNAL_COMMON_HPP
#define VSMC_RNG_INTERNAL_COMMON_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/simd.hpp>

#define VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS                                \
    param_type param() const { return param_; }                               \
    void param(const param_type &par) { param_ = par; }                       \
    void reset() { param_.reset(); }                                          \
                                                                              \
    friend bool operator==(                                                   \
        const distribution_type &dist1, const distribution_type &dist2)       \
    {                                                                         \
        return dist1.param_ == dist2.param_;                                  \
    }                                                                         \
                                                                              \
    friend bool operator!=(                                                   \
        const distribution_type &dist1, const distribution_type &dist2)       \
    {                                                                         \
        return dist1.param_ != dist2.param_;                                  \
    }                                                                         \
                                                                              \
    template <typename CharT, typename Traits>                                \
    friend std::basic_ostream<CharT, Traits> &operator<<(                     \
        std::basic_ostream<CharT, Traits> &os, const distribution_type &dist) \
    {                                                                         \
        os << dist.param_;                                                    \
                                                                              \
        return os;                                                            \
    }                                                                         \
                                                                              \
    template <typename CharT, typename Traits>                                \
    friend std::basic_istream<CharT, Traits> &operator>>(                     \
        std::basic_istream<CharT, Traits> &is, distribution_type &dist)       \
    {                                                                         \
        is >> dist.param_;                                                    \
                                                                              \
        return is;                                                            \
    }

namespace vsmc
{

namespace internal
{

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(KeyType, key_type, NullType)

template <typename T>
inline bool is_equal(const T &a, const T &b)
{
    return a == b;
}

inline bool is_equal(float a, float b)
{
    if (a > b || a < b)
        return false;
    return true;
}

inline bool is_equal(double a, double b)
{
    if (a > b || a < b)
        return false;
    return true;
}

template <int N>
class RNGBitsNMax
{
    public:
    static constexpr std::uint64_t
        value = std::numeric_limits<std::uint64_t>::max() >> (64 - N);
}; // class RNGBitsNMax

template <typename RNGType, int N>
class RNGBitsN
{
    static constexpr std::uint64_t umax = RNGType::max();
    static constexpr std::uint64_t bmax = RNGBitsNMax<N>::value;

    public:
    static constexpr int value =
        umax < bmax ? RNGBitsN<RNGType, N - 1>::value : N;
}; // class RNGBitsN

template <typename RNGType>
class RNGBitsN<RNGType, 0>
{
    public:
    static constexpr int value = 0;
}; // class RNGBitsN

template <typename RNGType>
class RNGBits : public RNGBitsN<RNGType, 64>
{
}; // class RNGBits

template <typename SeedSeq, typename U, typename V = U, typename W = V>
class is_seed_seq
    : public std::integral_constant<bool,
          !std::is_convertible<SeedSeq, U>::value &&
              !std::is_convertible<SeedSeq, V>::value &&
              !std::is_convertible<SeedSeq, W>::value &&
              !std::is_same<typename std::remove_cv<SeedSeq>::type,
                                        U>::value &&
              !std::is_same<typename std::remove_cv<SeedSeq>::type,
                                        V>::value &&
              !std::is_same<typename std::remove_cv<SeedSeq>::type, W>::value>
{
}; // class is_seed_seq

} // namespace vsmc::internal

/// \brief Generate random bits
/// \ingroup RNG
template <typename RNGType>
void rng_rand(RNGType &rng, std::size_t n, typename RNGType::result_type *r)
{
    for (std::size_t i = 0; i != n; ++i)
        r[i] = rng();
}

} // namespace vsmc

#endif // VSMC_RNG_INTERNAL_COMMON_HPP
