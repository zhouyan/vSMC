#ifndef VSMC_RNG_COMMON_HPP
#define VSMC_RNG_COMMON_HPP

#include <vsmc/internal/assert.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/forward.hpp>
#include <vsmc/cxx11/random.hpp>
#include <vsmc/cxx11/type_traits.hpp>
#include <vsmc/utility/static_vector.hpp>
#include <iostream>
#include <stdint.h>

namespace vsmc {

namespace internal {

template <std::size_t, std::size_t, std::size_t, typename ResultType>
inline void rng_array_left_assign (ResultType *, cxx11::false_type) {}

template <std::size_t N, std::size_t A, std::size_t I, typename ResultType>
inline void rng_array_left_assign (ResultType *state, cxx11::true_type)
{
    state[I] = state[I + A];
    rng_array_left_assign<N, A, I + 1>(state,
            cxx11::integral_constant<bool, (I + A + 1 < N)>());
}

template <std::size_t, std::size_t, typename ResultType>
inline void rng_array_left_zero (ResultType *, cxx11::false_type) {}

template <std::size_t N, std::size_t I, typename ResultType>
inline void rng_array_left_zero (ResultType *state, cxx11::true_type)
{
    state[I] = 0;
    rng_array_left_zero<N, I + 1>(state,
            cxx11::integral_constant<bool, (I + 1 < N)>());
}

template <std::size_t N, std::size_t A, bool fillzero, typename ResultType>
inline void rng_array_left_shift (ResultType *state)
{
    rng_array_left_assign<N, A, 0>(state,
            cxx11::integral_constant<bool, (A > 0 && A < N)>());
    rng_array_left_zero<N, N - A>(state,
            cxx11::integral_constant<bool, (fillzero && A > 0 && A <= N)>());
}

template <std::size_t, std::size_t, std::size_t, typename ResultType>
inline void rng_array_right_assign (ResultType *, cxx11::false_type) {}

template <std::size_t N, std::size_t A, std::size_t I, typename ResultType>
inline void rng_array_right_assign (ResultType *state, cxx11::true_type)
{
    state[I] = state[I - A];
    rng_array_right_assign<N, A, I - 1>(state,
            cxx11::integral_constant<bool, (A < I)>());
}

template <std::size_t, std::size_t, typename ResultType>
inline void rng_array_right_zero (ResultType *, cxx11::false_type) {}

template <std::size_t N, std::size_t I, typename ResultType>
inline void rng_array_right_zero (ResultType *state, cxx11::true_type)
{
    state[I] = 0;
    rng_array_right_zero<N, I - 1>(state,
            cxx11::integral_constant<bool, (I > 0)>());
}

template <std::size_t N, std::size_t A, bool fillzero, typename ResultType>
inline void rng_array_right_shift (ResultType *state)
{
    rng_array_right_assign<N, A, N - 1>(state,
            cxx11::integral_constant<bool, (A > 0 && A < N)>());
    rng_array_right_zero<N, A - 1>(state,
            cxx11::integral_constant<bool, (fillzero && A > 0 && A <= N)>());
}

template <typename SeedSeq, typename ResultType>
struct is_seed_sequence :
    public cxx11::integral_constant<bool,
    !cxx11::is_convertible<SeedSeq, ResultType>::value &&
    !cxx11::is_same<
    typename cxx11::remove_cv<SeedSeq>::type, ResultType>::value> {};

} // namespace vsmc::internal

} // namespace vsmc

#endif // VSMC_RNG_COMMON_HPP
