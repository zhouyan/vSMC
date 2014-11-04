//============================================================================
// include/vsmc/rng/internal/common.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_RNG_INTERNAL_COMMON_HPP
#define VSMC_RNG_INTERNAL_COMMON_HPP

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif

#include <vsmc/internal/common.hpp>
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
