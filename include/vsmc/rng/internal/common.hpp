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

#ifndef UINT64_C
#error __STDC_CONSTANT_MACROS not defined before #<stdint.h>
#endif

#define VSMC_DEFINE_RNG_DISTRIBUTION_OPERATORS                                \
    param_type param() const { return param_; }                               \
    void param(const param_type &par) { param_ = par; }                       \
    void reset() { param_.reset(); }                                          \
                                                                              \
    friend bool operator==(                                                   \
        const distribution_type &dist1, const distribution_type &dist2)       \
    {                                                                         \
        return dist1.param() == dist2.param();                                \
    }                                                                         \
                                                                              \
    friend bool operator!=(                                                   \
        const distribution_type &dist1, const distribution_type &dist2)       \
    {                                                                         \
        return dist1.param() != dist2.param();                                \
    }                                                                         \
                                                                              \
    template <typename CharT, typename Traits>                                \
    friend std::basic_ostream<CharT, Traits> &operator<<(                     \
        std::basic_ostream<CharT, Traits> &os, const distribution_type &dist) \
    {                                                                         \
        os << dist.param();                                                   \
                                                                              \
        return os;                                                            \
    }                                                                         \
                                                                              \
    template <typename CharT, typename Traits>                                \
    friend std::basic_istream<CharT, Traits> &operator>>(                     \
        std::basic_istream<CharT, Traits> &is, distribution_type &dist)       \
    {                                                                         \
        param_type param;                                                     \
        is >> param;                                                          \
        if (is.good())                                                        \
            dist.param(std::move(param));                                     \
                                                                              \
        return is;                                                            \
    }

namespace vsmc
{

namespace internal
{

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(KeyType, key_type, NullType)

template <typename RealType>
bool is_equal(RealType a, RealType b)
{
    if (a > b || a < b)
        return false;
    return true;
}

template <int N>
class RNGBitsNMax
{
    public:
    static constexpr std::uint64_t value = VSMC_MAX_UINT(std::uint64_t) >>
        (64 - N);
}; // class RNGBitsNMax

template <typename RNGType, int N>
class RNGBitsN
{
    static constexpr std::uint64_t umax = RNGType::max VSMC_MNE();
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

template <std::size_t, typename T, std::size_t K>
inline void increment_single(std::array<T, K> &ctr, std::false_type)
{
    ++ctr.back();
}

template <std::size_t N, typename T, std::size_t K>
inline void increment_single(std::array<T, K> &ctr, std::true_type)
{
    if (++std::get<N>(ctr) != 0)
        return;

    increment_single<N + 1>(ctr, std::integral_constant<bool, N + 2 < K>());
}

template <typename T, std::size_t K>
inline void increment(std::array<T, K> &, std::integral_constant<T, 0>)
{
}

template <typename T, std::size_t K, T NSkip>
inline void increment(std::array<T, K> &ctr, std::integral_constant<T, NSkip>)
{
    T old = ctr.front();
    ctr.front() += NSkip;
    if (ctr.front() > old)
        return;

    increment_single<1>(ctr, std::integral_constant<bool, 2 < K>());
}

template <typename T, std::size_t K>
inline void increment(std::array<T, K> &ctr, T nskip)
{
    T old = ctr.front();
    ctr.front() += nskip;
    if (ctr.front() > old || nskip == 0)
        return;

    increment_single<1>(ctr, std::integral_constant<bool, 2 < K>());
}

template <std::size_t Blocks, std::size_t, typename T, std::size_t K>
inline void increment_block(std::array<T, K> &,
    std::array<std::array<T, K>, Blocks> &, std::false_type)
{
}

template <std::size_t Blocks, std::size_t B, typename T, std::size_t K>
inline void increment_block(std::array<T, K> &ctr,
    std::array<std::array<T, K>, Blocks> &ctr_block, std::true_type)
{
    std::get<B>(ctr_block) = ctr;
    increment(std::get<B>(ctr_block), std::integral_constant<T, B + 1>());
    increment_block<Blocks, B + 1>(
        ctr, ctr_block, std::integral_constant<bool, B + 2 < Blocks>());
}

template <std::size_t Blocks, std::size_t, typename T, std::size_t K>
inline void increment_block_safe(std::array<T, K> &,
    std::array<std::array<T, K>, Blocks> &, std::false_type)
{
}

template <std::size_t Blocks, std::size_t B, typename T, std::size_t K>
inline void increment_block_safe(std::array<T, K> &ctr,
    std::array<std::array<T, K>, Blocks> &ctr_block, std::true_type)
{
    std::get<B>(ctr_block) = ctr;
    std::get<B>(ctr_block).front() += B + 1;
    increment_block_safe<Blocks, B + 1>(
        ctr, ctr_block, std::integral_constant<bool, B + 2 < Blocks>());
}

template <std::size_t Blocks, typename T, std::size_t K>
inline void increment_block_back(std::array<T, K> &ctr,
    std::array<std::array<T, K>, Blocks> &ctr_block, std::true_type)
{
    ctr_block.back() = ctr;
    T old = ctr_block.back().front();
    ctr_block.back().front() += Blocks;
    if (ctr_block.back().front() > old) {
        increment_block_safe<Blocks, 0>(
            ctr, ctr_block, std::integral_constant<bool, 1 < Blocks>());
    } else {
        increment_single<1>(
            ctr_block.back(), std::integral_constant<bool, 2 < K>());
        increment_block<Blocks, 0>(
            ctr, ctr_block, std::integral_constant<bool, 1 < Blocks>());
    }
    ctr = ctr_block.back();
}

template <typename T, std::size_t K>
inline void increment(std::array<T, K> &ctr)
{
    increment_single<0>(ctr, std::integral_constant<bool, 1 < K>());
}

template <std::size_t K>
inline void increment(std::array<std::uint32_t, K> &ctr)
{
    increment(*(reinterpret_cast<std::array<std::uint64_t, K / 2> *>(&ctr)));
}

template <std::size_t Blocks, typename T, std::size_t K>
inline void increment(
    std::array<T, K> &ctr, std::array<std::array<T, K>, Blocks> &ctr_block)
{
    increment_block_back<Blocks>(
        ctr, ctr_block, std::integral_constant<bool, 0 < Blocks>());
}

template <std::size_t Blocks, std::size_t K>
inline void increment(std::array<std::uint32_t, K> &ctr,
    std::array<std::array<std::uint32_t, K>, Blocks> &ctr_block)
{
    increment(*(reinterpret_cast<std::array<std::uint64_t, K / 2> *>(&ctr)),
        *(reinterpret_cast<std::array<std::array<std::uint64_t, K / 2>, Blocks>
                *>(&ctr_block)));
}

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
