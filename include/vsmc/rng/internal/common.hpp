//============================================================================
// vSMC/include/vsmc/rng/internal/common.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
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
#include <vsmc/utility/array.hpp>
#include <vsmc/utility/counter.hpp>

#ifndef UINT64_C
#error __STDC_CONSTANT_MACROS not defined before #<stdint.h>
#endif

namespace vsmc {

namespace internal {

template <std::size_t K, std::size_t, std::size_t, typename T>
inline void rng_array_left_assign (Array<T, K> &, cxx11::false_type) {}

template <std::size_t K, std::size_t A, std::size_t I, typename T>
inline void rng_array_left_assign (Array<T, K> &state, cxx11::true_type)
{
    state[Position<I>()] = state[Position<I + A>()];
    rng_array_left_assign<K, A, I + 1>(state,
            cxx11::integral_constant<bool, (I + A + 1 < K)>());
}

template <std::size_t K, std::size_t, typename T>
inline void rng_array_left_zero (Array<T, K> &, cxx11::false_type) {}

template <std::size_t K, std::size_t I, typename T>
inline void rng_array_left_zero (Array<T, K> &state, cxx11::true_type)
{
    state[Position<I>()] = 0;
    rng_array_left_zero<K, I + 1>(state,
            cxx11::integral_constant<bool, (I + 1 < K)>());
}

template <std::size_t K, std::size_t A, bool fillzero, typename T>
inline void rng_array_left_shift (Array<T, K> &state)
{
    rng_array_left_assign<K, A, 0>(state,
            cxx11::integral_constant<bool, (A > 0 && A < K)>());
    rng_array_left_zero<K, K - A>(state,
            cxx11::integral_constant<bool, (fillzero && A > 0 && A <= K)>());
}

template <std::size_t K, std::size_t, std::size_t, typename T>
inline void rng_array_right_assign (Array<T, K> &, cxx11::false_type) {}

template <std::size_t K, std::size_t A, std::size_t I, typename T>
inline void rng_array_right_assign (Array<T, K> &state, cxx11::true_type)
{
    state[Position<I>()] = state[Position<I - A>()];
    rng_array_right_assign<K, A, I - 1>(state,
            cxx11::integral_constant<bool, (A < I)>());
}

template <std::size_t K, std::size_t, typename T>
inline void rng_array_right_zero (Array<T, K> &, cxx11::false_type) {}

template <std::size_t K, std::size_t I, typename T>
inline void rng_array_right_zero (Array<T, K> &state, cxx11::true_type)
{
    state[Position<I>()] = 0;
    rng_array_right_zero<K, I - 1>(state,
            cxx11::integral_constant<bool, (I > 0)>());
}

template <std::size_t K, std::size_t A, bool fillzero, typename T>
inline void rng_array_right_shift (Array<T, K> &state)
{
    rng_array_right_assign<K, A, K - 1>(state,
            cxx11::integral_constant<bool, (A > 0 && A < K)>());
    rng_array_right_zero<K, A - 1>(state,
            cxx11::integral_constant<bool, (fillzero && A > 0 && A <= K)>());
}

template <typename SeedSeq, typename U, typename V = U, typename W = V>
struct is_seed_seq :
    public cxx11::integral_constant<bool,
    !cxx11::is_convertible<SeedSeq, U>::value &&
    !cxx11::is_convertible<SeedSeq, V>::value &&
    !cxx11::is_convertible<SeedSeq, W>::value &&
    !cxx11::is_same<typename cxx11::remove_cv<SeedSeq>::type, U>::value &&
    !cxx11::is_same<typename cxx11::remove_cv<SeedSeq>::type, V>::value &&
    !cxx11::is_same<typename cxx11::remove_cv<SeedSeq>::type, W>::value> {};

} // namespace vsmc::internal

} // namespace vsmc

#endif // VSMC_RNG_INTERNAL_COMMON_HPP
