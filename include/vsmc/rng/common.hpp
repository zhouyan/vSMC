#ifndef VSMC_RNG_COMMON_HPP
#define VSMC_RNG_COMMON_HPP

#include <vsmc/internal/assert.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/forward.hpp>
#include <vsmc/cxx11/random.hpp>
#include <vsmc/cxx11/type_traits.hpp>
#include <vsmc/utility/static_vector.hpp>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdint.h>

#ifndef UINT64_C
#define DEFINE MACRO __STDC_CONSTANT_MACROS BEFORE INCLUDING <stdint.h>
#endif

#ifndef VSMC_INT64
#if defined(__INTEL_COMPILER) || defined(_MSC_VER)
#define VSMC_INT64  __int64
#else
#define VSMC_INT64  long long
#endif
#endif // VSMC_INT64

#ifdef __x86_64__
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
#endif // __x86_64__

#define VSMC_STATIC_ASSERT_RNG_COUNTER_RESULT(K, Blocsk, N) \
    VSMC_STATIC_ASSERT((K * Blocks == N),                                    \
            USE_RngCounter_result_WITH_INCOMPATIBLE_TYPES)

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

template <typename T>
class RngCounter
{
    public :

    template <std::size_t K, typename Traits>
    static const StaticVector<T, K, Traits> &get (
            const StaticVector<T, K, Traits> &ctr) {return ctr;}

    template <std::size_t K, typename Traits,
             std::size_t Blocks, typename BlockTraits>
    static const StaticVector<T, K, Traits> &get (
            const StaticVector<StaticVector<T, K, Traits>,
            Blocks, BlockTraits> &ctr) {return ctr.back();}

    template <std::size_t K, typename Traits>
    static void set (StaticVector<T, K, Traits> &ctr,
            const StaticVector<T, K, Traits> &c) {ctr = c;}

    template <std::size_t K, typename Traits,
             std::size_t Blocks, typename BlockTraits>
    static void set (StaticVector<StaticVector<T, K, Traits>,
            Blocks, BlockTraits> &ctr, const StaticVector<T, K, Traits> &c)
    {
        ctr.front() = c;
        increment_block<1>(ctr, cxx11::integral_constant<bool, 1 < Blocks>());
    }

    template <std::size_t K, typename Traits>
    static void reset (StaticVector<T, K, Traits> &ctr) {ctr.fill(0);}

    template <std::size_t K, typename Traits,
             std::size_t Blocks, typename BlockTraits>
    static void reset (StaticVector<StaticVector<T, K, Traits>,
            Blocks, BlockTraits> &ctr)
    {
        ctr.front().fill(0);
        increment_block<1>(ctr, cxx11::integral_constant<bool, 1 < Blocks>());
    }

    template <std::size_t K, typename Traits, typename ResultTraits>
    static void result (const StaticVector<T, K, Traits> &ctr,
            StaticVector<T, K, ResultTraits> &res) {res = ctr;}

    template <std::size_t K, typename Traits,
             std::size_t Blocks, typename BlockTraits,
             std::size_t N, typename ResultTraits>
    static void result (const StaticVector<StaticVector<T, K, Traits>,
            Blocks, BlockTraits> &ctr, StaticVector<T, N, ResultTraits> &res)
    {
        VSMC_STATIC_ASSERT_RNG_COUNTER_RESULT(K, Blocsk, N);
        result<0>(ctr, res, cxx11::integral_constant<bool, 0 < N>());
    }

    template <std::size_t K, typename Traits>
    static void increment (StaticVector<T, K, Traits> &ctr)
    {increment_ctr<0>(ctr, cxx11::integral_constant<bool, 0 < K>());}

    template <std::size_t K, typename Traits>
    static void increment (StaticVector<T, K, Traits> &ctr, std::size_t nskip)
    {
        if (K == 0)
            return;

        if (nskip == 0)
            return;

        if (nskip == 1) {
            increment(ctr);
            return;
        }

        uint64_t nskip_64 = static_cast<uint64_t>(nskip);
        const uint64_t max_64 = static_cast<uint64_t>(max_);
        if (nskip_64 / K > max_64)
            nskip_64 %= (K * max_64);

        while (nskip_64 > max_64) {
            increment_ctr<0>(ctr, max_,
                    cxx11::integral_constant<bool, 0 < K>());
            nskip_64 -= max_64;
        }
        increment_ctr<0>(ctr, static_cast<T>(nskip_64),
                cxx11::true_type());
    }

    template <std::size_t K, typename Traits,
             std::size_t Blocks, typename BlockTraits>
    static void increment (StaticVector<StaticVector<T, K, Traits>,
            Blocks, BlockTraits> &ctr)
    {
        ctr.front() = ctr.back();
        increment(ctr.front());
        increment_block<1>(ctr, cxx11::integral_constant<bool, 1 < Blocks>());
    }

    template <std::size_t K, typename Traits,
             std::size_t Blocks, typename BlockTraits>
    static void increment (StaticVector<StaticVector<T, K, Traits>,
            Blocks, BlockTraits> &ctr, std::size_t nskip)
    {
        if (K == 0 || Blocks == 0)
            return;

        if (nskip == 0)
            return;

        if (nskip == 1) {
            increment(ctr);
            return;
        }

        increment(ctr.back(), (nskip - 1) * Blocks);
        increment(ctr);
    }

    private :

    template <std::size_t, std::size_t K, typename Traits,
             std::size_t Blocks, typename BlockTraits,
             std::size_t N, typename ResultTraits>
    static void result (const StaticVector<StaticVector<T, K, Traits>,
            Blocks, BlockTraits> &, StaticVector<T, N, ResultTraits> &,
            cxx11::false_type) {}

    template <std::size_t B, std::size_t K, typename Traits,
             std::size_t Blocks, typename BlockTraits,
             std::size_t N, typename ResultTraits>
    static void result (const StaticVector<StaticVector<T, K, Traits>,
            Blocks, BlockTraits> &ctr, StaticVector<T, N, ResultTraits> &res,
            cxx11::true_type)
    {
        std::memcpy(res.data() + B * K, ctr[Position<B>()].data(),
                K * sizeof(T));
        result<B + 1>(ctr, res,
                cxx11::integral_constant<bool, B + 1 < Blocks>());
    }

    static VSMC_CONSTEXPR const T max_ = static_cast<T>(
            ~(static_cast<T>(0)));

    template <std::size_t, std::size_t K, typename Traits>
    static void increment_ctr (StaticVector<T, K, Traits> &ctr,
            cxx11::false_type) {ctr.fill(0);}

    template <std::size_t N, std::size_t K, typename Traits>
    static void increment_ctr (StaticVector<T, K, Traits> &ctr,
            cxx11::true_type)
    {
        if (ctr[Position<N>()] < max_) {
            ++ctr[Position<N>()];
            return;
        }
        increment_ctr<N + 1>(ctr, cxx11::integral_constant<bool, N + 1 < K>());
    }

    template <std::size_t, std::size_t K, typename Traits>
    static void increment_ctr (StaticVector<T, K, Traits> &ctr, T nskip,
            cxx11::false_type)
    {
        if (nskip == 0)
            return;

        ctr.fill(0);
        ctr.front() = nskip - 1;
    }

    template <std::size_t N, std::size_t K, typename Traits>
    static void increment_ctr (StaticVector<T, K, Traits> &ctr, T nskip,
            cxx11::true_type)
    {
        if (nskip == 0)
            return;

        const T remain = max_ - ctr[Position<N>()];
        if (nskip <= remain) {
            ctr[Position<N>()] += nskip;
            return;
        }
        ctr[Position<N>()] = max_;
        increment_ctr<N + 1>(ctr, nskip - remain,
                cxx11::integral_constant<bool, N + 1 < K>());
    }

    template <std::size_t, std::size_t K, typename Traits,
             std::size_t Blocks, typename BlockTraits>
    static void increment_block (StaticVector<StaticVector<T, K, Traits>,
            Blocks, BlockTraits> &, cxx11::false_type) {}

    template <std::size_t B, std::size_t K, typename Traits,
             std::size_t Blocks, typename BlockTraits>
    static void increment_block (StaticVector<StaticVector<T, K, Traits>,
            Blocks, BlockTraits> &ctr, cxx11::true_type)
    {
        ctr[Position<B>()] = ctr[Position<B - 1>()];
        increment(ctr[Position<B>()]);
        increment_block<B + 1>(ctr,
                cxx11::integral_constant<bool, B + 1 < Blocks>());
    }
}; // struct RngCounter

template <typename SeedSeq, typename T>
struct is_seed_seq :
    public cxx11::integral_constant<bool,
    !cxx11::is_convertible<SeedSeq, T>::value &&
    !cxx11::is_same<typename cxx11::remove_cv<SeedSeq>::type, T>::value> {};

} // namespace vsmc::internal

} // namespace vsmc

#endif // VSMC_RNG_COMMON_HPP
