#ifndef VSMC_RNG_INTERNAL_COMMON_HPP
#define VSMC_RNG_INTERNAL_COMMON_HPP

#include <vsmc/cxx11/random.hpp>
#include <vsmc/utility/array.hpp>
#include <vsmc/utility/counter.hpp>
#include <stdint.h>

#ifndef UINT64_C
#error __STDC_CONSTANT_MACROS not defined before #<stdint.h>
#endif

#ifndef VSMC_INT64
#if defined(__INTEL_COMPILER) || defined(_MSC_VER)
#define VSMC_INT64  __int64
#else
#define VSMC_INT64  long long
#endif
#endif // VSMC_INT64

#if defined(__x86_64__) || defined(_M_AMD64) || defined (_M_X64)
#ifndef VSMC_HAS_INT128
#undef VSMC_INT128
#if defined(__INTEL_COMPILER)
#define VSMC_HAS_INT128 1
#define VSMC_INT128  __int128
#elif defined(__clang__)
#define VSMC_HAS_INT128 1
#define VSMC_INT128  __int128
#elif defined(__OPEN64__)
#define VSMC_HAS_INT128 0
#elif defined(__SUNPRO_CC)
#define VSMC_HAS_INT128 0
#elif defined(__GNUC__)
#define VSMC_HAS_INT128 1
#define VSMC_INT128  __int128
#elif defined(_MSC_VER)
#define VSMC_HAS_INT128 0
#else
#define VSMC_HAS_INT128 0
#endif
#endif // VSMC_HAS_INT128
#endif // defined(__x86_64__) || defined(_M_AMD64) || defined (_M_X64)

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
