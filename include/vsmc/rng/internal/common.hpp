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
#include <vsmc/utility/aligned_memory.hpp>
#include <vsmc/utility/counter.hpp>

#ifndef UINT64_C
#error __STDC_CONSTANT_MACROS not defined before #<stdint.h>
#endif

namespace vsmc
{

namespace traits
{

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(KeyType, key_type, void)

} // namespace vsmc::traits

namespace internal
{

template <std::size_t K, std::size_t, std::size_t, typename T>
inline void rng_array_left_assign(std::array<T, K> &, std::false_type)
{
}

template <std::size_t K, std::size_t A, std::size_t I, typename T>
inline void rng_array_left_assign(std::array<T, K> &state, std::true_type)
{
    std::get<I>(state) = std::get<I + A>(state);
    rng_array_left_assign<K, A, I + 1>(
        state, std::integral_constant<bool, (I + A + 1 < K)>());
}

template <std::size_t K, std::size_t, typename T>
inline void rng_array_left_zero(std::array<T, K> &, std::false_type)
{
}

template <std::size_t K, std::size_t I, typename T>
inline void rng_array_left_zero(std::array<T, K> &state, std::true_type)
{
    std::get<I>(state) = 0;
    rng_array_left_zero<K, I + 1>(
        state, std::integral_constant<bool, (I + 1 < K)>());
}

template <std::size_t K, std::size_t A, bool fillzero, typename T>
inline void rng_array_left_shift(std::array<T, K> &state)
{
    rng_array_left_assign<K, A, 0>(
        state, std::integral_constant<bool, (A > 0 && A < K)>());
    rng_array_left_zero<K, K - A>(
        state, std::integral_constant<bool, (fillzero && A > 0 && A <= K)>());
}

template <std::size_t K, std::size_t, std::size_t, typename T>
inline void rng_array_right_assign(std::array<T, K> &, std::false_type)
{
}

template <std::size_t K, std::size_t A, std::size_t I, typename T>
inline void rng_array_right_assign(std::array<T, K> &state, std::true_type)
{
    std::get<I>(state) = std::get<I - A>(state);
    rng_array_right_assign<K, A, I - 1>(
        state, std::integral_constant<bool, (A < I)>());
}

template <std::size_t K, std::size_t, typename T>
inline void rng_array_right_zero(std::array<T, K> &, std::false_type)
{
}

template <std::size_t K, std::size_t I, typename T>
inline void rng_array_right_zero(std::array<T, K> &state, std::true_type)
{
    std::get<I>(state) = 0;
    rng_array_right_zero<K, I - 1>(
        state, std::integral_constant<bool, (I > 0)>());
}

template <std::size_t K, std::size_t A, bool fillzero, typename T>
inline void rng_array_right_shift(std::array<T, K> &state)
{
    rng_array_right_assign<K, A, K - 1>(
        state, std::integral_constant<bool, (A > 0 && A < K)>());
    rng_array_right_zero<K, A - 1>(
        state, std::integral_constant<bool, (fillzero && A > 0 && A <= K)>());
}

template <typename SeedSeq, typename U, typename V = U, typename W = V>
struct is_seed_seq
    : public std::integral_constant<bool,
          !std::is_convertible<SeedSeq, U>::value &&
              !std::is_convertible<SeedSeq, V>::value &&
              !std::is_convertible<SeedSeq, W>::value &&
              !std::is_same<typename std::remove_cv<SeedSeq>::type,
                  U>::value &&
              !std::is_same<typename std::remove_cv<SeedSeq>::type,
                  V>::value &&
              !std::is_same<typename std::remove_cv<SeedSeq>::type,
                  W>::value> {
};

template <typename T, std::size_t K>
using ctr_type_8 =
    typename std::conditional<sizeof(T) * K % sizeof(std::uint8_t) == 0,
        std::array<std::uint8_t, sizeof(T) * K / sizeof(std::uint8_t)>,
        std::array<T, K>>::type;

template <typename T, std::size_t K>
using ctr_type_16 =
    typename std::conditional<sizeof(T) * K % sizeof(std::uint16_t) == 0,
        std::array<std::uint16_t, sizeof(T) * K / sizeof(std::uint16_t)>,
        ctr_type_8<T, K>>::type;

template <typename T, std::size_t K>
using ctr_type_32 =
    typename std::conditional<sizeof(T) * K % sizeof(std::uint32_t) == 0,
        std::array<std::uint32_t, sizeof(T) * K / sizeof(std::uint32_t)>,
        ctr_type_16<T, K>>::type;

template <typename T, std::size_t K>
using ctr_type_64 =
    typename std::conditional<sizeof(T) * K % sizeof(std::uint64_t) == 0,
        std::array<std::uint64_t, sizeof(T) * K / sizeof(std::uint64_t)>,
        ctr_type_32<T, K>>::type;

#if VSMC_HAS_INT128
template <typename T, std::size_t K>
using ctr_type_128 =
    typename std::conditional<sizeof(T) * K % sizeof(VSMC_INT128) == 0,
        std::array<unsigned VSMC_INT128, sizeof(T) * K / sizeof(VSMC_INT128)>,
        ctr_type_64<T, K>>::type;

template <typename T, std::size_t K>
using ctr_type_max = ctr_type_128<T, K>;
#else
template <typename T, std::size_t K>
using ctr_type_max = ctr_type_64<T, K>;
#endif

} // namespace vsmc::internal

} // namespace vsmc

#endif // VSMC_RNG_INTERNAL_COMMON_HPP
