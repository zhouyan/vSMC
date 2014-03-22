#ifndef VSMC_RNG_INTERNAL_COMMON_HPP
#define VSMC_RNG_INTERNAL_COMMON_HPP

#include <vsmc/cxx11/random.hpp>
#include <vsmc/utility/array.hpp>
#include <vsmc/utility/counter.hpp>
#include <stdint.h>

#ifndef UINT64_C
#error __STDC_CONSTANT_MACROS not defined before #<stdint.h>
#endif

namespace vsmc {

namespace internal {

template <std::size_t K, std::size_t, std::size_t,
         typename T, typename Traits>
inline void rng_array_left_assign (Array<T, K, Traits> &, cxx11::false_type) {}

template <std::size_t K, std::size_t A, std::size_t I,
         typename T, typename Traits>
inline void rng_array_left_assign (Array<T, K, Traits> &state,
        cxx11::true_type)
{
    state[Position<I>()] = state[Position<I + A>()];
    rng_array_left_assign<K, A, I + 1>(state,
            cxx11::integral_constant<bool, (I + A + 1 < K)>());
}

template <std::size_t K, std::size_t, typename T, typename Traits>
inline void rng_array_left_zero (Array<T, K, Traits> &, cxx11::false_type) {}

template <std::size_t K, std::size_t I, typename T, typename Traits>
inline void rng_array_left_zero (Array<T, K, Traits> &state, cxx11::true_type)
{
    state[Position<I>()] = 0;
    rng_array_left_zero<K, I + 1>(state,
            cxx11::integral_constant<bool, (I + 1 < K)>());
}

template <std::size_t K, std::size_t A, bool fillzero,
        typename T, typename Traits>
inline void rng_array_left_shift (Array<T, K, Traits> &state)
{
    rng_array_left_assign<K, A, 0>(state,
            cxx11::integral_constant<bool, (A > 0 && A < K)>());
    rng_array_left_zero<K, K - A>(state,
            cxx11::integral_constant<bool, (fillzero && A > 0 && A <= K)>());
}

template <std::size_t K, std::size_t, std::size_t,
         typename T, typename Traits>
inline void rng_array_right_assign (Array<T, K, Traits> &,
        cxx11::false_type) {}

template <std::size_t K, std::size_t A, std::size_t I,
        typename T, typename Traits>
inline void rng_array_right_assign (Array<T, K, Traits> &state,
        cxx11::true_type)
{
    state[Position<I>()] = state[Position<I - A>()];
    rng_array_right_assign<K, A, I - 1>(state,
            cxx11::integral_constant<bool, (A < I)>());
}

template <std::size_t K, std::size_t, typename T, typename Traits>
inline void rng_array_right_zero (Array<T, K, Traits> &, cxx11::false_type) {}

template <std::size_t K, std::size_t I, typename T, typename Traits>
inline void rng_array_right_zero (Array<T, K, Traits> &state,
        cxx11::true_type)
{
    state[Position<I>()] = 0;
    rng_array_right_zero<K, I - 1>(state,
            cxx11::integral_constant<bool, (I > 0)>());
}

template <std::size_t K, std::size_t A, bool fillzero,
        typename T, typename Traits>
inline void rng_array_right_shift (Array<T, K, Traits> &state)
{
    rng_array_right_assign<K, A, K - 1>(state,
            cxx11::integral_constant<bool, (A > 0 && A < K)>());
    rng_array_right_zero<K, A - 1>(state,
            cxx11::integral_constant<bool, (fillzero && A > 0 && A <= K)>());
}

template <typename SeedSeq, typename T>
struct is_seed_seq :
    public cxx11::integral_constant<bool,
    !cxx11::is_convertible<SeedSeq, T>::value &&
    !cxx11::is_same<typename cxx11::remove_cv<SeedSeq>::type, T>::value> {};

} // namespace vsmc::internal

} // namespace vsmc

#endif // VSMC_RNG_INTERNAL_COMMON_HPP
