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

template <typename> struct RngUIntBits;

template <> struct RngUIntBits<uint32_t> :
    public cxx11::integral_constant<std::size_t, 32> {};

template <> struct RngUIntBits<uint64_t> :
    public cxx11::integral_constant<std::size_t, 64> {};

template <typename ResultType, unsigned N> struct RngRotate;

template <unsigned N>
struct RngRotate<uint32_t, N>
{
    static uint32_t rotate (uint32_t x)
    {return (x << (N & 31)) | (x >> ((32 - N) & 31));}
};

template <unsigned N>
struct RngRotate<uint64_t, N>
{
    static uint64_t rotate (uint64_t x)
    {return (x << (N & 63)) | (x >> ((64 - N) & 63));}
};

template <typename, std::size_t> struct RngCounterIncrement;

template <typename ResultType, std::size_t K>
struct RngCounterIncrement
{
    static void increment (ResultType *ctr)
    {increment<0>(ctr, cxx11::true_type());}

    private :

    static VSMC_CONSTEXPR const ResultType max_ = static_cast<ResultType>(
            ~(static_cast<ResultType>(0)));

    template <std::size_t>
    static void increment (ResultType *ctr, cxx11::false_type)
    {
        for (std::size_t i = 0; i != K; ++i)
            ctr[i] = 0;
    }

    template <std::size_t N>
    static void increment (ResultType *ctr, cxx11::true_type)
    {
        if (ctr[N] < max_) {
            ++ctr[N];
            return;
        }

        increment<N + 1>(ctr, cxx11::integral_constant<bool, N < K>());
    }
};

template <typename SeedSeq, typename ResultType>
struct is_seed_sequence :
    public cxx11::integral_constant<bool,
    !cxx11::is_convertible<SeedSeq, ResultType>::value &&
    !cxx11::is_same<
    typename cxx11::remove_cv<SeedSeq>::type, ResultType>::value> {};

} // namespace vsmc::internal

} // namespace vsmc

#endif // VSMC_RNG_COMMON_HPP
