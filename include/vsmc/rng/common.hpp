#ifndef VSMC_RNG_COMMON_HPP
#define VSMC_RNG_COMMON_HPP

#include <vsmc/internal/assert.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/forward.hpp>
#include <vsmc/cxx11/random.hpp>
#include <vsmc/cxx11/type_traits.hpp>
#include <vsmc/utility/static_vector.hpp>
#include <iomanip>
#include <iostream>
#include <stdint.h>

#ifndef UINT64_C
#define DEFINE MACRO __STDC_CONSTANT_MACROS BEFORE INCLUDING <stdint.h>
#endif

#ifndef VSMC_HAS_INT128
#undef VSMC_INT128
#undef VSMC_UINT128
#if defined(__INTEL_COMPILER)
#define VSMC_HAS_INT128 1
#define VSMC_INT128  __int128_t
#define VSMC_UINT128 __uint128_t
#elif defined(__clang__)
#define VSMC_HAS_INT128 1
#define VSMC_INT128  __int128_t
#define VSMC_UINT128 __uint128_t
#elif defined(__OPEN64__)
#define VSMC_HAS_INT128 0
#elif defined(__SUNPRO_CC)
#define VSMC_HAS_INT128 0
#elif defined(__GNUC__)
#ifdef __x86_64__
#define VSMC_HAS_INT128 1
#define VSMC_INT128  __int128_t
#define VSMC_UINT128 __uint128_t
#else
#define VSMC_HAS_INT128 0
#endif
#elif defined(_MSC_VER)
#define VSMC_HAS_INT128 0
#endif
#endif

#ifndef VSMC_HAS_INT128
#define VSMC_HAS_INT128 0
#endif

namespace vsmc {

namespace internal {

template <std::size_t K, std::size_t, std::size_t,
         typename T, typename Traits>
inline void rng_array_left_assign (StaticVector<T, K, Traits> &,
        cxx11::false_type) {}

template <std::size_t K, std::size_t A, std::size_t I,
         typename T, typename Traits>
inline void rng_array_left_assign (StaticVector<T, K, Traits> &state,
        cxx11::true_type)
{
    state[Position<I>()] = state[Position<I + A>()];
    rng_array_left_assign<K, A, I + 1>(state,
            cxx11::integral_constant<bool, (I + A + 1 < K)>());
}

template <std::size_t K, std::size_t, typename T, typename Traits>
inline void rng_array_left_zero (StaticVector<T, K, Traits> &,
        cxx11::false_type) {}

template <std::size_t K, std::size_t I, typename T, typename Traits>
inline void rng_array_left_zero (StaticVector<T, K, Traits> &state,
        cxx11::true_type)
{
    state[Position<I>()] = 0;
    rng_array_left_zero<K, I + 1>(state,
            cxx11::integral_constant<bool, (I + 1 < K)>());
}

template <std::size_t K, std::size_t A, bool fillzero,
        typename T, typename Traits>
inline void rng_array_left_shift (StaticVector<T, K, Traits> &state)
{
    rng_array_left_assign<K, A, 0>(state,
            cxx11::integral_constant<bool, (A > 0 && A < K)>());
    rng_array_left_zero<K, K - A>(state,
            cxx11::integral_constant<bool, (fillzero && A > 0 && A <= K)>());
}

template <std::size_t K, std::size_t, std::size_t,
         typename T, typename Traits>
inline void rng_array_right_assign (StaticVector<T, K, Traits> &,
        cxx11::false_type) {}

template <std::size_t K, std::size_t A, std::size_t I,
        typename T, typename Traits>
inline void rng_array_right_assign (StaticVector<T, K, Traits> &state,
        cxx11::true_type)
{
    state[Position<I>()] = state[Position<I - A>()];
    rng_array_right_assign<K, A, I - 1>(state,
            cxx11::integral_constant<bool, (A < I)>());
}

template <std::size_t K, std::size_t, typename T, typename Traits>
inline void rng_array_right_zero (StaticVector<T, K, Traits> &,
        cxx11::false_type) {}

template <std::size_t K, std::size_t I, typename T, typename Traits>
inline void rng_array_right_zero (StaticVector<T, K, Traits> &state,
        cxx11::true_type)
{
    state[Position<I>()] = 0;
    rng_array_right_zero<K, I - 1>(state,
            cxx11::integral_constant<bool, (I > 0)>());
}

template <std::size_t K, std::size_t A, bool fillzero,
        typename T, typename Traits>
inline void rng_array_right_shift (StaticVector<T, K, Traits> &state)
{
    rng_array_right_assign<K, A, K - 1>(state,
            cxx11::integral_constant<bool, (A > 0 && A < K)>());
    rng_array_right_zero<K, A - 1>(state,
            cxx11::integral_constant<bool, (fillzero && A > 0 && A <= K)>());
}

template <typename T, std::size_t K>
struct RngCounter
{
    template <typename Traits>
    static void increment (StaticVector<T, K, Traits> &ctr)
    {increment<0>(ctr, cxx11::true_type());}

    template <typename Traits>
    static void increment (StaticVector<T, K, Traits> &ctr, std::size_t nskip)
    {
        uint64_t nskip_64 = static_cast<uint64_t>(nskip);
        uint64_t max_64 = static_cast<uint64_t>(max_);
        while (nskip_64 > max_64) {
            increment<0>(ctr, max_, cxx11::true_type());
            nskip_64 -= max_64;
        }
        increment<0>(ctr, static_cast<T>(nskip_64),
                cxx11::true_type());
    }

    private :

    static VSMC_CONSTEXPR const T max_ = static_cast<T>(
            ~(static_cast<T>(0)));

    template <std::size_t, typename Traits>
    static void increment (StaticVector<T, K, Traits> &ctr, cxx11::false_type)
    {ctr.fill(0);}

    template <std::size_t N, typename Traits>
    static void increment (StaticVector<T, K, Traits> &ctr, cxx11::true_type)
    {
        if (ctr[N] < max_) {
            ++ctr[N];
            return;
        }

        increment<N + 1>(ctr, cxx11::integral_constant<bool, N < K>());
    }

    template <std::size_t, typename Traits>
    static void increment (StaticVector<T, K, Traits> &ctr, T nskip,
            cxx11::false_type)
    {
        if (nskip == 0)
            return;

        ctr.fill(0);
        --nskip;
        ctr[0] = nskip;
    }

    template <std::size_t N, typename Traits>
    static void increment (StaticVector<T, K, Traits> &ctr, T nskip,
            cxx11::true_type)
    {
        if (nskip <= max_ - ctr[N]) {
            ctr[N] += nskip;
            return;
        }
        ctr[N] = max_;
        increment<N + 1>(ctr, nskip - (max_ - ctr[N]),
                cxx11::integral_constant<bool, N < K>());
    }
};

template <typename SeedSeq, typename T>
struct is_seed_sequence :
    public cxx11::integral_constant<bool,
    !cxx11::is_convertible<SeedSeq, T>::value &&
    !cxx11::is_same<typename cxx11::remove_cv<SeedSeq>::type, T>::value> {};

} // namespace vsmc::internal

} // namespace vsmc

#endif // VSMC_RNG_COMMON_HPP
